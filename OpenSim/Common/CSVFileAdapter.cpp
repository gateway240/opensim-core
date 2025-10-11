#include "CSVFileAdapter.h"
#include "OpenSim/Common/DataAdapter.h"
#include <memory>

namespace OpenSim {

CSVFileAdapter::CSVFileAdapter() :
    DelimFileAdapter(",", // delimiter for read
                     ","  // delimiter for write
                     ) {}

std::unique_ptr<DataAdapter>
CSVFileAdapter::clone() const {
    return std::make_unique<CSVFileAdapter>(*this);
}

void 
CSVFileAdapter::write(const TimeSeriesTable& table, 
                        const std::string& fileName) {
    InputTables tables{};
    tables.emplace(tableString(), &table);
    CSVFileAdapter{}.extendWrite(tables, fileName);
}

}
