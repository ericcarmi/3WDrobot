#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

using namespace visualization_msgs;
using namespace interactive_markers;
using namespace std;
boost::shared_ptr<InteractiveMarkerServer> server;
float marker_pos = 0;

MenuHandler menu_handler;

MenuHandler::EntryHandle h_first_entry;
MenuHandler::EntryHandle h_mode_last;

string current = "sine220.wav";

int numlocations = 0;

void enableCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  menu_handler.getCheckState( handle, state );

  if ( state == MenuHandler::CHECKED )
  {
    menu_handler.setCheckState( handle, MenuHandler::UNCHECKED );
    ROS_INFO("Hiding first menu entry");
    menu_handler.setVisible( h_first_entry, false );

  }
  else
  {
    menu_handler.setCheckState( handle, MenuHandler::CHECKED );
    ROS_INFO("Showing first menu entry");
    menu_handler.setVisible( h_first_entry, true );
  }
  menu_handler.reApply( *server );
  ros::Duration(2.0).sleep();
  ROS_INFO("update");
  server->applyChanges();
}

void modeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  menu_handler.setCheckState( h_mode_last, MenuHandler::UNCHECKED );
  h_mode_last = feedback->menu_entry_id;
  menu_handler.setCheckState( h_mode_last, MenuHandler::CHECKED );

  ROS_INFO("Switching to menu entry #%d", h_mode_last);

  menu_handler.reApply( *server );
  server->applyChanges();
}



Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

Marker makeBoxAt( InteractiveMarker &msg, float x, float y, float t, float z )
{
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = cos(t);
  marker.pose.orientation.y = sin(t);
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

InteractiveMarker makeEmptyMarker( bool dummyBox=true )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
//  int_marker.pose.position.y = -3.0 * marker_pos++;;
  int_marker.scale = 1;

  return int_marker;
}

void makeMenuMarker( std::string name )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = name;

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  control.markers.push_back( makeBox( int_marker ) );
  int_marker.controls.push_back(control);

  server->insert( int_marker );
}

void makeMenuMarkerAt( std::string name, float x, float y, float t, float z )
{
  InteractiveMarker int_marker = makeEmptyMarker();
  int_marker.name = name;

  InteractiveMarkerControl control;

  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.always_visible = true;

  control.markers.push_back( makeBoxAt( int_marker, x, y, t, z ) );
  int_marker.controls.push_back(control);

  server->insert( int_marker );
}

void publishLoop()
{
  ros::NodeHandle n;
  ros::Publisher positionPublisher = n.advertise<std_msgs::String>("mapPub", 1000);
  ros::Rate loop_rate(1);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = current;

    positionPublisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}

void deepCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO("The deep sub-menu has been found.");
  current = (feedback->marker_name).c_str();
  ROS_INFO(current.c_str());
  publishLoop();
}

void initMenu()
{
  h_first_entry = menu_handler.insert( "First Entry" );
//  MenuHandler::EntryHandle entry = menu_handler.insert( h_first_entry, "deep" );
//  entry = menu_handler.insert( entry, "sub" );
//  entry = menu_handler.insert( entry, "menu", &deepCb );
  MenuHandler::EntryHandle entry = menu_handler.insert( h_first_entry, "plot", &deepCb );

  menu_handler.setCheckState( menu_handler.insert( "Show First Entry", &enableCb ), MenuHandler::CHECKED );

  MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Switch" );

  for ( int i=0; i<5; i++ )
  {
    std::ostringstream s;
    s << "Mode " << i;
    h_mode_last = menu_handler.insert( sub_menu_handle, s.str(), &modeCb );
    menu_handler.setCheckState( h_mode_last, MenuHandler::UNCHECKED );
  }
  //check the very last entry
  menu_handler.setCheckState( h_mode_last, MenuHandler::CHECKED );
}

void stringto4nums(string s, float state[5])
{
  string delimiter = ",";
  size_t pos = 0;
  std::string token;
  int i = 0;

  while ((pos = s.find(delimiter)) != std::string::npos) {
      token = s.substr(0, pos);
      state[i] = std::stof(s);
      i += 1;
      s.erase(0, pos + delimiter.length());

  }
  state[i] = std::stof(s);
}

// On callback, place marker at coordinates given in the String
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  string s = msg->data.c_str();

  float state[5];
  stringto4nums(s,state);

  // Use constant name for robot's position
  makeMenuMarkerAt("robot", state[0], state[1], state[2], 0);
  menu_handler.apply( *server, "robot" );

  if(state[4] == 1)
  {
    makeMenuMarkerAt(s, state[0], state[1], state[2], state[3]+5);
    numlocations += 1;
    std::vector<string>newlocations(numlocations);
    menu_handler.apply( *server, s );
  }
  server->applyChanges();
  std::string outs = std::to_string(state[0]) +","+  std::to_string(state[1]) +","+ std::to_string(state[2]) +","+ std::to_string(state[3]) +","+ std::to_string(state[4]);
  ROS_INFO(outs.c_str());

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "menu");

  server.reset( new InteractiveMarkerServer("menu","",false) );
  ros::NodeHandle n;
  ros::Subscriber positionSubscriber = n.subscribe("mapSub",1000, chatterCallback);

  ros::Publisher positionPublisher = n.advertise<std_msgs::String>("mapPub", 1000);

  initMenu();
  server->applyChanges();

  ros::Rate loop_rate(1);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = current;

    positionPublisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}
