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
#include <map>
#include <set>
#include <string>

namespace OpenSim {

XsensDataReader* XsensDataReader::clone() const {
    return new XsensDataReader{*this};
}

DataAdapter::OutputTables XsensDataReader::extendRead(
        const std::string& folderName) const {

    // This encapsulates all the accepted headers and their groups
    const static std::map<std::string, std::set<std::string>>
            _accepted_headers = {{"accelerometer", {"Acc_X", "Acc_Y", "Acc_Z"}},
                    {"gyroscope", {"Gyr_X", "Gyr_Y", "Gyr_Z"}},
                    {"magnetometer", {"Mag_X", "Mag_Y", "Mag_Z"}},
                    {"rot_quaternion",
                            {"Quat_q0", "Quat_q1", "Quat_q2", "Quat_q3"}},
                    {"rot_euler", {"Roll", "Pitch", "Yaw"}},
                    {"rot_matrix",
                            {"Mat[1][1]", "Mat[2][1]", "Mat[3][1]", "Mat[1][2]",
                                    "Mat[2][2]", "Mat[3][2]", "Mat[1][3]",
                                    "Mat[2][3]", "Mat[3][3]"}}};

    std::vector<std::unique_ptr<std::ifstream>> imuStreams;
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
    std::map<std::string, size_t> h_map;

    for (int index = 0; index < n_imus; ++index) {
        std::string prefix = _settings.get_trial_prefix();
        const ExperimentalSensor& nextItem =
                _settings.get_ExperimentalSensors(index);
        const std::filesystem::path fileName =
                std::filesystem::path(folderName) /
                (prefix + nextItem.getName() + extension);
        auto nextStream = std::make_unique<std::ifstream>(fileName);

        OPENSIM_THROW_IF(!nextStream->good(), FileDoesNotExist, fileName);

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
        // Process each header in the line
        for (auto& pair : _accepted_headers) {
            const std::string& group = pair.first;
            const std::set<std::string>& headers = pair.second;
            for (const auto& header : headers) {
                int tokenIndex = std::distance(tokens.begin(),
                        std::find(tokens.begin(), tokens.end(), header));
                if (tokenIndex != -1) { h_map.insert({header, tokenIndex}); }
            }
        }

        // Count the total lines in the file
        // TODO: make sure all the sensor files have the same number of lines
        const int fp_pos = nextStream->tellg();
        // std::cout << "Current pos: " << fp_pos << std::endl;
        // Count the number of lines
        n_lines = std::count(std::istreambuf_iterator<char>(*nextStream),
                std::istreambuf_iterator<char>(), '\n');
        // Rewind file pointer to after header
        nextStream->seekg(fp_pos, std::ifstream::beg);
        // std::cout << "Number of Lines: " << n_lines << std::endl;

        // Add imu name to labels
        labels.push_back(nextItem.get_name_in_model());
        // Add corresponding stream to imuStreams
        imuStreams.push_back(std::move(nextStream));
    }
    // Compute data rate based on key/value pair if available
    std::map<std::string, std::string>::iterator it =
            headersKeyValuePairs.find("Update Rate");
    if (it != headersKeyValuePairs.end())
        dataRate = OpenSim::IO::stod(it->second);
    // Make sure that the specified header group has all required headers
    auto is_group_complete =
            [&](const std::string& group,
                    const std::map<std::string, std::set<std::string>>&
                            accepted_headers,
                    const std::map<std::string, size_t>& found_headers)
            -> bool {
        const auto& reqIt = accepted_headers.find(group);
        if (reqIt == accepted_headers.end()) return false;
        const auto& search_set = reqIt->second;
        return std::all_of(
                search_set.begin(), search_set.end(), [&](const auto& p) {
                    return found_headers.find(p) != found_headers.end();
                });
    };
    // internally keep track of what data was found in input files
    bool foundLinearAccelerationData =
            is_group_complete("accelerometer", _accepted_headers, h_map);
    bool foundMagneticHeadingData =
            is_group_complete("magnetometer", _accepted_headers, h_map);
    bool foundAngularVelocityData =
            is_group_complete("gyroscope", _accepted_headers, h_map);
    bool foundRotationData = is_group_complete(
            rotation_representation, _accepted_headers, h_map);
    // If no Orientation data is available we'll abort completely
    OPENSIM_THROW_IF(!foundRotationData, TableMissingHeader,
            "Rotation Data not found. Please ensure that the "
            "XsensDataReaderSettings match the file format being parsed!\n"
            " Attempted to parse with rotation format: \"" +
                    rotation_representation_str + "\" and delimiter: \"" +
                    delimiter + "\"");

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
        for (auto&& nextStream : imuStreams) {
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
            imu_index++;
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

} // namespace OpenSim
