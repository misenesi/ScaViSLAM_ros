
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include "anchored_points.h"

#include <iostream>

namespace g2o {
  using namespace std;

  bool init_g2o_types()
  {
    cerr << __PRETTY_FUNCTION__ << " called" << endl;
    Factory* factory = Factory::instance();
    factory->registerType("G2O_CAMERA_PARAMETERS", new HyperGraphElementCreator<ScaViSLAM::G2oCameraParameters>);
    factory->registerType("G2O_VERTEX_SE3", new HyperGraphElementCreator<ScaViSLAM::G2oVertexSE3>);
    factory->registerType("G2O_VERTEX_POINT_XYZ", new HyperGraphElementCreator<ScaViSLAM::G2oVertexPointXYZ>);
    factory->registerType("G2O_EDGE_PROJECT_PSI2UVU", new HyperGraphElementCreator<ScaViSLAM::G2oEdgeProjectPSI2UVU>);
    factory->registerType("G2O_EDGE_SE3", new HyperGraphElementCreator<ScaViSLAM::G2oEdgeSE3>);
    return true;
  }

} // end namespace
