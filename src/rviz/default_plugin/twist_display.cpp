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

#include "twist_visual.h"

#include "twist_display.h"

namespace rviz
{
TwistStampedDisplay::TwistStampedDisplay()
{
  linear_color_property_ =
      new rviz::ColorProperty("Linear Color", QColor(204, 51, 51), "Color to draw the linear arrows.",
                              this, SLOT(updateProperties()));

  angular_color_property_ =
      new rviz::ColorProperty("Angular Color", QColor(204, 204, 51), "Color to draw the angular arrows.",
                              this, SLOT(updateProperties()));

  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
                                            this, SLOT(updateProperties()));

  linear_scale_property_ = new rviz::FloatProperty("Linear Arrow Scale", 2.0, "linear arrow scale", this,
                                                   SLOT(updateProperties()));

  angular_scale_property_ = new rviz::FloatProperty("Angular Arrow Scale", 2.0, "angular arrow scale",
                                                    this, SLOT(updateProperties()));

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

void TwistStampedDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

TwistStampedDisplay::~TwistStampedDisplay()
{
}

// Override rviz::Display's reset() function to add a call to clear().
void TwistStampedDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void TwistStampedDisplay::updateProperties()
{
  float alpha = alpha_property_->getFloat();
  float linear_scale = linear_scale_property_->getFloat();
  float angular_scale = angular_scale_property_->getFloat();
  float width = width_property_->getFloat();
  bool hide_small_values = hide_small_values_property_->getBool();
  Ogre::ColourValue linear_color = linear_color_property_->getOgreColor();
  Ogre::ColourValue angular_color = angular_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setLinearColor(linear_color.r, linear_color.g, linear_color.b, alpha);
    visuals_[i]->setAngularColor(angular_color.r, angular_color.g, angular_color.b, alpha);
    visuals_[i]->setLinearScale(linear_scale);
    visuals_[i]->setAngularScale(angular_scale);
    visuals_[i]->setWidth(width);
    visuals_[i]->setHideSmallValues(hide_small_values);
  }
}

// Set the number of past visuals to show.
void TwistStampedDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}


bool validateFloats(const geometry_msgs::TwistStamped& msg)
{
  return rviz::validateFloats(msg.twist.linear) && rviz::validateFloats(msg.twist.angular);
}

// This is our callback to handle an incoming message.
void TwistStampedDisplay::processMessage(const geometry_msgs::TwistStamped::ConstPtr& msg)
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
        1.0, "Twist position contains NaNs. Skipping render as long as the position is invalid");
    return;
  }

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<TwistVisual> visual;
  if (visuals_.full())
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new TwistVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual->setTwist(msg->twist);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);
  float alpha = alpha_property_->getFloat();
  float linear_scale = linear_scale_property_->getFloat();
  float angular_scale = angular_scale_property_->getFloat();
  float width = width_property_->getFloat();
  Ogre::ColourValue linear_color = linear_color_property_->getOgreColor();
  Ogre::ColourValue angular_color = angular_color_property_->getOgreColor();
  visual->setLinearColor(linear_color.r, linear_color.g, linear_color.b, alpha);
  visual->setAngularColor(angular_color.r, angular_color.g, angular_color.b, alpha);
  visual->setLinearScale(linear_scale);
  visual->setAngularScale(angular_scale);
  visual->setWidth(width);

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

} // end namespace rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::TwistStampedDisplay, rviz::Display)
