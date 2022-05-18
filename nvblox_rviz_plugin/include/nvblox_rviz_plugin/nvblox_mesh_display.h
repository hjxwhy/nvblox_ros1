
#include <memory>

#include <rviz/message_filter_display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <nvblox_msgs/Mesh.h>

#include "nvblox_rviz_plugin/nvblox_mesh_visual.h"

namespace nvblox_rviz_plugin {

class NvbloxMeshVisual;

class NvbloxMeshDisplay
    : public rviz::MessageFilterDisplay<nvblox_msgs::Mesh> {
  Q_OBJECT
 public:
  NvbloxMeshDisplay();
  virtual ~NvbloxMeshDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

 private:
  void processMessage(const nvblox_msgs::Mesh::ConstPtr& msg);

  rviz::BoolProperty* cut_ceiling_property_;
  rviz::FloatProperty* ceiling_height_property_;
  

  std::unique_ptr<NvbloxMeshVisual> visual_;
};

}  // namespace voxblox_rviz_plugin

