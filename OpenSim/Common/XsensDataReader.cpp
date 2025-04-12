#include "XsensDataReader.h"

#include "CommonUtilities.h"
#include "Exception.h"
#include "FileAdapter.h"
#include "Simbody.h"
#include "TimeSeriesTable.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace OpenSim {

XsensDataReader* XsensDataReader::clone() const {
    return new XsensDataReader{*this};
}

const std::map<std::string, std::set<std::string>> XsensDataReader::_accepted_headers = {
        {"accelerometer", {"Acc_X", "Acc_Y", "Acc_Z"}},
        {"gyroscope", {"Gyr_X", "Gyr_Y", "Gyr_Z"}},
        {"magnetometer", {"Mag_X", "Mag_Y", "Mag_Z"}},
        {"rot_quaternion", {"Quat_q0", "Quat_q1", "Quat_q2", "Quat_q3"}},
        {"rot_euler", {"Roll", "Pitch", "Yaw"}},
        {"rot_matrix", {"Mat[1][1]", "Mat[2][1]", "Mat[3][1]", "Mat[1][2]",
                               "Mat[2][2]", "Mat[3][2]", "Mat[1][3]",
                               "Mat[2][3]", "Mat[3][3]"}}};

DataAdapter::OutputTables XsensDataReader::extendRead(
        const std::string& folderName) const {

    std::vector<std::ifstream*> imuStreams;
    std::vector<std::string> labels;
    // files specified by prefix + file name exist
    double dataRate = _settings.get_sampling_rate();
    const std::string delimiter = _settings.get_delimiter();
    const std::string extension = _settings.get_trial_extension();
    const std::string rotation_representation_str =
            _settings.get_rotation_representation();
    const std::string rotation_representation = rotation_representation_str;

    int n_imus = _settings.getProperty_ExperimentalSensors().size();
    int n_lines = 1;

    std::string prefix = _settings.get_trial_prefix();
    std::map<std::string, std::string> headersKeyValuePairs;
    // Map to store the headers and their column indices by group
    std::map<std::string, std::set<std::pair<std::string, size_t>>>
            presentGroupsWithColumnIndices;

    std::map<std::string, size_t> h_map;
    for (int index = 0; index < n_imus; ++index) {
        std::string prefix = _settings.get_trial_prefix();
        const ExperimentalSensor& nextItem =
                _settings.get_ExperimentalSensors(index);
        const std::filesystem::path fileName =
                std::filesystem::path(folderName) /
                (prefix + nextItem.getName() + extension);
        auto* nextStream = new std::ifstream{fileName};

        OPENSIM_THROW_IF(!nextStream->good(), FileDoesNotExist, fileName);
        // Add imu name to labels
        labels.push_back(nextItem.get_name_in_model());
        // Add corresponding stream to imuStreams
        imuStreams.push_back(nextStream);

        // Skip lines to get to data
        std::string line;
        auto commentLine = true;
        std::getline(*nextStream, line);
        auto isCommentLine = [](std::string aline) {
            return aline.substr(0, 2) == "//";
        };
        std::vector<std::string> tokens;
        while (commentLine) {
            // Comment lines of arbitrary number on the form // "key":"value"
            // Skip leading 2 chars tokenize on ':'
            tokens = FileAdapter::tokenize(line.substr(2), ":");
            // Will populate map from first imu only!
            // Assumes they all have same format
            if (tokens.size() == 2 && index == 0) {
                // Put values in map
                headersKeyValuePairs[tokens[0]] = tokens[1];
            }
            std::getline(*nextStream, line);
            commentLine = isCommentLine(line);
        }

        // This is the header
        // Find indices for Acc_{X,Y,Z}, Gyr_{X,Y,Z},
        // Mag_{X,Y,Z}, Mat on first non-comment line
        tokens = FileAdapter::tokenize(line, delimiter);
        // Process each word in the line (separated by spaces)
        for (auto& pair : _accepted_headers) {
            const std::string& group = pair.first;
            const std::set<std::string>& headers = pair.second;
            for (const auto& header : headers) {
                int tokenIndex = find_index(tokens, header);
                if (tokenIndex != -1) {
                    // Add the header with its column index to the map
                    presentGroupsWithColumnIndices[group].insert(
                            {header, tokenIndex});
                }
            }
        }
        // Output the present header groups and their column indices
        // std::cout << "Present header groups with their column indices:"
        //           << std::endl;

        for (const auto& group : presentGroupsWithColumnIndices) {
            // std::cout << group.first << ":" << std::endl;
            for (const auto& header : group.second) {
                // std::cout << "  " << header.first << " at column "
                //           << header.second << std::endl;
                h_map.insert({header.first, header.second});
            }
        }
        // Count the total lines in the file
        // TODO: make sure all the sensor files have the same number of lines
        std::ifstream lineCount{fileName};
        n_lines = std::count(std::istreambuf_iterator<char>(lineCount),
                std::istreambuf_iterator<char>(), '\n');
        // std::cout << "Number of Lines: " << n_lines << std::endl;
    }
    // Compute data rate based on key/value pair if available
    std::map<std::string, std::string>::iterator it =
            headersKeyValuePairs.find("Update Rate");
    if (it != headersKeyValuePairs.end())
        dataRate = OpenSim::IO::stod(it->second);
    // internally keep track of what data was found in input files
    bool foundLinearAccelerationData = is_group_complete(
            "accelerometer", _accepted_headers, presentGroupsWithColumnIndices);
    bool foundMagneticHeadingData = is_group_complete(
            "magnetometer", _accepted_headers, presentGroupsWithColumnIndices);
    bool foundAngularVelocityData = is_group_complete(
            "gyroscope", _accepted_headers, presentGroupsWithColumnIndices);
    bool foundRotationData = is_group_complete(rotation_representation,
        _accepted_headers, presentGroupsWithColumnIndices);
    // If no Orientation data is available we'll abort completely
    OPENSIM_THROW_IF(!foundRotationData, TableMissingHeader);

    // Will read data into pre-allocated Matrices in-memory rather than
    // appendRow on the fly to avoid the overhead of
    SimTK::Matrix_<SimTK::Quaternion> rotationsData{n_lines, n_imus};
    SimTK::Matrix_<SimTK::Vec3> linearAccelerationData{n_lines, n_imus};
    SimTK::Matrix_<SimTK::Vec3> magneticHeadingData{n_lines, n_imus};
    SimTK::Matrix_<SimTK::Vec3> angularVelocityData{n_lines, n_imus};

    // For all tables, will create row, stitch values from different files then
    // append,time and timestep are based on the first file
    bool end_of_file = false;
    int rowNumber = 0;
    while (!end_of_file) {
        // Make vectors one per table
        TimeSeriesTableQuaternion::RowVector orientation_row_vector{
                n_imus, SimTK::Quaternion()};
        TimeSeriesTableVec3::RowVector accel_row_vector{
                n_imus, SimTK::Vec3(SimTK::NaN)};
        TimeSeriesTableVec3::RowVector magneto_row_vector{
                n_imus, SimTK::Vec3(SimTK::NaN)};
        TimeSeriesTableVec3::RowVector gyro_row_vector{
                n_imus, SimTK::Vec3(SimTK::NaN)};
        // Cycle through the files collating values
        int imu_index = 0;
        for (std::vector<std::ifstream*>::iterator it = imuStreams.begin();
                it != imuStreams.end(); ++it, ++imu_index) {
            std::ifstream* nextStream = *it;
            // parse gyro info from imuStream
            std::vector<std::string> nextRow =
                    FileAdapter::getNextLine(*nextStream, delimiter + "\r");
            if (nextRow.empty()) {
                end_of_file = true;
                break;
            }
            if (foundLinearAccelerationData)
                accel_row_vector[imu_index] = SimTK::Vec3(
                        OpenSim::IO::stod(nextRow[h_map.at("Acc_X")]),
                        OpenSim::IO::stod(nextRow[h_map.at("Acc_Y")]),
                        OpenSim::IO::stod(nextRow[h_map.at("Acc_Z")]));
            if (foundMagneticHeadingData)
                magneto_row_vector[imu_index] = SimTK::Vec3(
                        OpenSim::IO::stod(nextRow[h_map.at("Gyr_X")]),
                        OpenSim::IO::stod(nextRow[h_map.at("Gyr_Y")]),
                        OpenSim::IO::stod(nextRow[h_map.at("Gyr_Z")]));
            if (foundAngularVelocityData)
                gyro_row_vector[imu_index] = SimTK::Vec3(
                        OpenSim::IO::stod(nextRow[h_map.at("Mag_X")]),
                        OpenSim::IO::stod(nextRow[h_map.at("Mag_Y")]),
                        OpenSim::IO::stod(nextRow[h_map.at("Mag_Z")]));
            if (foundRotationData) {
                if (rotation_representation == "rot_quaternion") {
                    orientation_row_vector[imu_index] = SimTK::Quaternion(
                            OpenSim::IO::stod(nextRow[h_map.at("Quat_q0")]),
                            OpenSim::IO::stod(nextRow[h_map.at("Quat_q1")]),
                            OpenSim::IO::stod(nextRow[h_map.at("Quat_q2")]),
                            OpenSim::IO::stod(nextRow[h_map.at("Quat_q3")]));
                } else if (rotation_representation == "rot_euler") {
                    const auto rot = SimTK::Rotation(
                            SimTK::BodyOrSpaceType::BodyRotationSequence,
                            OpenSim::IO::stod(nextRow[h_map.at("Roll")]),
                            SimTK::XAxis,
                            OpenSim::IO::stod(nextRow[h_map.at("Pitch")]),
                            SimTK::YAxis,
                            OpenSim::IO::stod(nextRow[h_map.at("Yaw")]),
                            SimTK::ZAxis);
                    orientation_row_vector[imu_index] =
                            rot.convertRotationToQuaternion();
                } else if (rotation_representation == "rot_matrix") {
                    // Create Mat33 then convert into Quaternion
                    SimTK::Mat33 imu_matrix{SimTK::NaN};
                    int matrix_entry_index = 0;
                    imu_matrix[0][0] =
                            OpenSim::IO::stod(nextRow[h_map.at("Mat[1][1]")]);
                    imu_matrix[1][0] =
                            OpenSim::IO::stod(nextRow[h_map.at("Mat[2][1]")]);
                    imu_matrix[2][0] =
                            OpenSim::IO::stod(nextRow[h_map.at("Mat[3][1]")]);

                    imu_matrix[0][1] =
                            OpenSim::IO::stod(nextRow[h_map.at("Mat[1][2]")]);
                    imu_matrix[1][1] =
                            OpenSim::IO::stod(nextRow[h_map.at("Mat[2][2]")]);
                    imu_matrix[2][1] =
                            OpenSim::IO::stod(nextRow[h_map.at("Mat[3][2]")]);

                    imu_matrix[0][2] =
                            OpenSim::IO::stod(nextRow[h_map.at("Mat[1][3]")]);
                    imu_matrix[1][2] =
                            OpenSim::IO::stod(nextRow[h_map.at("Mat[2][3]")]);
                    imu_matrix[2][2] =
                            OpenSim::IO::stod(nextRow[h_map.at("Mat[3][3]")]);

                    // Convert imu_matrix to Quaternion
                    SimTK::Rotation imu_rotation{imu_matrix};
                    orientation_row_vector[imu_index] =
                            imu_rotation.convertRotationToQuaternion();
                }
            }
        }
        if (end_of_file) { break; }
        // append to the tables
        if (foundLinearAccelerationData)
            linearAccelerationData[rowNumber] = accel_row_vector;
        if (foundMagneticHeadingData)
            magneticHeadingData[rowNumber] = magneto_row_vector;
        if (foundAngularVelocityData)
            angularVelocityData[rowNumber] = gyro_row_vector;
        rotationsData[rowNumber] = orientation_row_vector;
        rowNumber++;
    }
    const double timeIncrement = 1.0 / dataRate;
    const auto times =
            createVectorLinspaceInterval(rowNumber, 0.0, timeIncrement);

    // Repeat for Data matrices in use and create Tables from them or size 0 for
    // empty
    linearAccelerationData.resizeKeep(
            foundLinearAccelerationData ? rowNumber : 0, n_imus);
    magneticHeadingData.resizeKeep(
            foundMagneticHeadingData ? rowNumber : 0, n_imus);
    angularVelocityData.resizeKeep(
            foundAngularVelocityData ? rowNumber : 0, n_imus);
    rotationsData.resizeKeep(rowNumber, n_imus);

    // Now create the tables from matrices
    // Create 4 tables for Rotations, LinearAccelerations, AngularVelocity,
    // MagneticHeading Tables could be empty if data is not present in file(s)
    DataAdapter::OutputTables tables = createTablesFromMatrices(dataRate,
            labels, times, rotationsData, linearAccelerationData,
            magneticHeadingData, angularVelocityData);
    return tables;
}

bool XsensDataReader::is_group_complete(const std::string& group,
        const std::map<std::string, std::set<std::string>>&
                headers,
        const std::map<std::string, std::set<std::pair<std::string, size_t>>>&
                presentGroupsWithColumnIndices) {
    auto reqIt = headers.find(group);
    if (reqIt == headers.end()) return false;

    auto presIt = presentGroupsWithColumnIndices.find(group);
    if (presIt == presentGroupsWithColumnIndices.end()) return false;

    std::set<std::string> present;
    for (const auto& pair : presIt->second) { present.insert(pair.first); }

    for (const auto& header : reqIt->second) {
        if (present.find(header) == present.end()) { return false; }
    }

    return true;
}

int XsensDataReader::find_index(
        std::vector<std::string>& tokens, const std::string& keyToMatch) {
    int returnIndex = -1;
    std::vector<std::string>::iterator it =
            std::find(tokens.begin(), tokens.end(), keyToMatch);
    if (it != tokens.end())
        returnIndex = static_cast<int>(std::distance(tokens.begin(), it));
    return returnIndex;
}
} // namespace OpenSim
