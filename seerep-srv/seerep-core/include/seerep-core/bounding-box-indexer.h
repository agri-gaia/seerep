#ifndef SEEREP_CORE_BB_INDEXER_H_
#define SEEREP_CORE_BB_INDEXER_H_

#include <functional>
#include <optional>

namespace seerep_core
{
class BoundingBoxIndexer {
public:
    BoundingBoxIndexer();
    virtual getData(boundingBox BB, labels labels) = 0;

private:

};


} /* namespace seerep_core */

#endif // SEEREP_CORE_BB_INDEXER_H_
