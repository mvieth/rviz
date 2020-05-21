#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <ros/ros.h>

#include "vector3_visual.h"

namespace rviz
{
Vector3Visual::Vector3Visual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Vector3Stamped's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();
  vector_node_ = frame_node_->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  arrow_vector_ = new rviz::Arrow(scene_manager_, vector_node_);
}

Vector3Visual::~Vector3Visual()
{
  // Delete the arrow to make it disappear.
  delete arrow_vector_;

  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}


void Vector3Visual::setVector3(const geometry_msgs::Vector3& vector3)
{
  Ogre::Vector3 vector(vector3.x, vector3.y, vector3.z);
  setVector3(vector);
}

void Vector3Visual::setVector3(const Ogre::Vector3& vector)
{
  double vector_length = vector.length() * vector_scale_;
  // hide markers if they get too short and hide_small_values_ is activated
  // "too short" is defined as "vector_length > width_"
  bool show_vector = (vector_length > width_) || !hide_small_values_;

  if (show_vector)
  {
    arrow_vector_->setScale(Ogre::Vector3(vector_length, width_, width_));
    arrow_vector_->setDirection(vector);
  }
  vector_node_->setVisible(show_vector);
}

// Position and orientation are passed through to the SceneNode.
void Vector3Visual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void Vector3Visual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

// Color is passed through to the rviz object.
void Vector3Visual::setVectorColor(float r, float g, float b, float a)
{
  arrow_vector_->setColor(r, g, b, a);
}

void Vector3Visual::setVectorScale(float s)
{
  vector_scale_ = s;
}

void Vector3Visual::setWidth(float w)
{
  width_ = w;
}

void Vector3Visual::setHideSmallValues(bool h)
{
  hide_small_values_ = h;
}


void Vector3Visual::setVisible(bool visible)
{
  frame_node_->setVisible(visible);
}

} // end namespace rviz
