#ifndef VECTOR3STAMPED_VISUAL_H
#define VECTOR3STAMPED_VISUAL_H

#include <geometry_msgs/Vector3.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Arrow;
class BillboardLine;
}

namespace rviz
{
// Each instance of Vector3StampedVisual represents the visualization of a single
// sensor_msgs::Vector3Stamped message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class Vector3Visual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  Vector3Visual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~Vector3Visual();

  // Configure the visual to show the given vectors
  void setVector3(const Ogre::Vector3& vector);
  // Configure the visual to show the data in the message.
  void setVector3(const geometry_msgs::Vector3& vector3);

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way Vector3StampedVisual is only
  // responsible for visualization.
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Vector3Stamped message.
  void setVectorColor(float r, float g, float b, float a);
  void setVectorScale(float s);
  void setWidth(float w);
  void setHideSmallValues(bool h);
  void setVisible(bool visible);

private:
  // The object implementing the vector3Stamped circle
  rviz::Arrow* arrow_vector_;
  float vector_scale_, width_;
  bool hide_small_values_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Vector3Stamped message header.
  Ogre::SceneNode* frame_node_;
  // allow showing/hiding of vector arrow
  Ogre::SceneNode* vector_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
};

} // end namespace rviz

#endif // VECTOR3STAMPED_VISUAL_H
