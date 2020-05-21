#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/parse_color.h>
#include <rviz/validate_floats.h>

#include <boost/foreach.hpp>

#include "vector3_visual.h"

#include "vector3_display.h"

namespace rviz
{
Vector3StampedDisplay::Vector3StampedDisplay()
{
  force_color_property_ =
      new rviz::ColorProperty("Force Color", QColor(204, 51, 51), "Color to draw the force arrows.",
                              this, SLOT(updateProperties()));

  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
                                            this, SLOT(updateProperties()));

  force_scale_property_ = new rviz::FloatProperty("Force Arrow Scale", 2.0, "force arrow scale", this,
                                                  SLOT(updateProperties()));

  width_property_ =
      new rviz::FloatProperty("Arrow Width", 0.5, "arrow width", this, SLOT(updateProperties()));


  history_length_property_ =
      new rviz::IntProperty("History Length", 1, "Number of prior measurements to display.", this,
                            SLOT(updateHistoryLength()));

  hide_small_values_property_ = new rviz::BoolProperty("Hide Small Values", true, "Hide small values",
                                                       this, SLOT(updateProperties()));

  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

void Vector3StampedDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

Vector3StampedDisplay::~Vector3StampedDisplay()
{
}

// Override rviz::Display's reset() function to add a call to clear().
void Vector3StampedDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void Vector3StampedDisplay::updateProperties()
{
  float alpha = alpha_property_->getFloat();
  float force_scale = force_scale_property_->getFloat();
  float width = width_property_->getFloat();
  bool hide_small_values = hide_small_values_property_->getBool();
  Ogre::ColourValue force_color = force_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setForceColor(force_color.r, force_color.g, force_color.b, alpha);
    visuals_[i]->setForceScale(force_scale);
    visuals_[i]->setWidth(width);
    visuals_[i]->setHideSmallValues(hide_small_values);
  }
}

// Set the number of past visuals to show.
void Vector3StampedDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}


bool validateFloats(const geometry_msgs::Vector3Stamped& msg)
{
  return rviz::validateFloats(msg.vector);
}

// This is our callback to handle an incoming message.
void Vector3StampedDisplay::processMessage(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  if (!validateFloats(*msg))
  {
    setStatus(rviz::StatusProperty::Error, "Topic",
              "Message contained invalid floating point values (nans or infs)");
    return;
  }

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position,
                                                 orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  if (position.isNaN())
  {
    ROS_ERROR_THROTTLE(
        1.0, "Vector3 position contains NaNs. Skipping render as long as the position is invalid");
    return;
  }

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<Vector3Visual> visual;
  if (visuals_.full())
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new Vector3Visual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual->setVector3(msg->vector);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);
  float alpha = alpha_property_->getFloat();
  float force_scale = force_scale_property_->getFloat();
  float width = width_property_->getFloat();
  Ogre::ColourValue force_color = force_color_property_->getOgreColor();
  visual->setForceColor(force_color.r, force_color.g, force_color.b, alpha);
  visual->setForceScale(force_scale);
  visual->setWidth(width);

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

} // end namespace rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::Vector3StampedDisplay, rviz::Display)
