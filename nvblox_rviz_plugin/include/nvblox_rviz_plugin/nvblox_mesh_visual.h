#ifndef NVBLOX_RVIZ_PLUGIN_NVBLOX_MESH_VISUAL_H_
#define NVBLOX_RVIZ_PLUGIN_NVBLOX_MESH_VISUAL_H_

#include <OGRE/OgreManualObject.h>

// #include <voxblox/core/block_hash.h>
#include <nvblox_msgs/Mesh.h>
#include "nvblox_rviz_plugin/nvblox_hash_utils.h"

namespace nvblox_rviz_plugin {

/// Visualizes a single voxblox_msgs::Mesh message.
class NvbloxMeshVisual {
 public:
  NvbloxMeshVisual(Ogre::SceneManager* scene_manager,
                    Ogre::SceneNode* parent_node);
  virtual ~NvbloxMeshVisual();

  void setMessage(const nvblox_msgs::Mesh::ConstPtr& msg);

  /// Set the coordinate frame pose.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setCeilingCutoff(bool cut_ceiling, float ceiling_height);

 private:
  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;

  unsigned int instance_number_;
  static unsigned int instance_counter_;

  bool cut_ceiling_ = false;
  float ceiling_height_ = 0.0f;

  float block_size_ = 0.0f;

  nvblox_rviz_plugin::Index3DHashMapType<Ogre::ManualObject*>::type object_map_;

};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_VISUAL_H_
