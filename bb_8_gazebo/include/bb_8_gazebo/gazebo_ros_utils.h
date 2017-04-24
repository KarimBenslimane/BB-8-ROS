/*********************************************************************
  * Software License Agreement (BSD License)
  *
  *  Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
  *  All rights reserved.
  *
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *
  *   * Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *   * Redistributions in binary form must reproduce the above
  *     copyright notice, this list of conditions and the following
  *     disclaimer in the documentation and/or other materials provided
  *     with the distribution.
  *   * Neither the name of the copyright holder nor the names of its
  *     contributors may be used to endorse or promote products derived from
  *     this software without specific prior written permission.
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *********************************************************************/
 
 #ifndef GAZEBO_ROS_UTILS_H
 #define GAZEBO_ROS_UTILS_H
 #include <map>
 #include <boost/algorithm/string.hpp>
 
 #include <gazebo/common/common.hh>
 #include <gazebo/physics/physics.hh>
 #include <gazebo/sensors/Sensor.hh>
 #include <gazebo/gazebo_config.h>
 #include <ros/ros.h>
 
 #ifndef GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST
 # if GAZEBO_MAJOR_VERSION >= 7
 #define GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST using std::dynamic_pointer_cast
 # else
 #define GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST using boost::dynamic_pointer_cast
 # endif
 #endif
 
 namespace gazebo
 {
 
 inline std::string GetModelName ( const sensors::SensorPtr &parent )
 {
     std::string modelName;
     std::vector<std::string> values;
     std::string scopedName = parent->ScopedName();
     boost::replace_all ( scopedName, "::", "," );
     boost::split ( values, scopedName, boost::is_any_of ( "," ) );
     if ( values.size() < 2 ) {
         modelName = "";
     } else {
         modelName = values[1];
     }
     return modelName;
 }
 
 inline std::string GetRobotNamespace ( const sensors::SensorPtr &parent, const sdf::ElementPtr &sdf, const char *pInfo = NULL )
 {
     std::string name_space;
     std::stringstream ss;
     if ( sdf->HasElement ( "robotNamespace" ) ) {
         name_space = sdf->Get<std::string> ( "robotNamespace" );
         if ( name_space.empty() ) {
             ss << "The 'robotNamespace' param was empty";
             name_space = GetModelName ( parent );
         } else {
             ss << "Using the 'robotNamespace' param: '" <<  name_space << "'";
         }
     } else {
         ss << "The 'robotNamespace' param did not exit";
     }
     if ( pInfo != NULL ) {
         ROS_INFO_NAMED("utils", "%s Plugin: %s" , pInfo, ss.str().c_str() );
     }
     return name_space;
 }
 
 class GazeboRos
 {
 private:
     sdf::ElementPtr sdf_;       
     std::string plugin_;        
     std::string namespace_;     
     boost::shared_ptr<ros::NodeHandle> rosnode_; 
     std::string tf_prefix_;     
     std::string info_text;      
 
     void readCommonParameter ();
 public:
     GazeboRos ( physics::ModelPtr &_parent, sdf::ElementPtr _sdf, const std::string &_plugin )
         : sdf_ ( _sdf ), plugin_ ( _plugin ) {
         namespace_ = _parent->GetName ();
         if ( !sdf_->HasElement ( "robotNamespace" ) ) {
             ROS_INFO_NAMED("utils", "%s missing <robotNamespace>, defaults is %s", plugin_.c_str(), namespace_.c_str() );
         }  else {
             namespace_ = sdf_->GetElement ( "robotNamespace" )->Get<std::string>();
             if ( namespace_.empty() ) {
                 namespace_ = _parent->GetName();
             }
         }
         if ( !namespace_.empty() )
             this->namespace_ += "/";
         rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( namespace_ ) );
         info_text = plugin_ + "(ns = " + namespace_ + ")";
         readCommonParameter ();
     }
     GazeboRos ( sensors::SensorPtr _parent, sdf::ElementPtr _sdf, const std::string &_plugin )
         : sdf_ ( _sdf ), plugin_ ( _plugin ) {
 
         std::stringstream ss;
         if ( sdf_->HasElement ( "robotNamespace" ) ) {
             namespace_ = sdf_->Get<std::string> ( "robotNamespace" );
             if ( namespace_.empty() ) {
                 ss << "the 'robotNamespace' param was empty";
                 namespace_ = GetModelName ( _parent );
             } else {
                 ss << "Using the 'robotNamespace' param: '" <<  namespace_ << "'";
             }
         } else {
             ss << "the 'robotNamespace' param did not exit";
         }
         info_text = plugin_ + "(ns = " + namespace_ + ")";
         ROS_INFO_NAMED("utils", "%s: %s" , info_text.c_str(), ss.str().c_str() );
         readCommonParameter ();
     }
 
     const char* info() const;
     boost::shared_ptr<ros::NodeHandle>& node();;
     const boost::shared_ptr<ros::NodeHandle>& node() const;
     std::string resolveTF ( const std::string &_name );
 
     void getParameterBoolean ( bool &_value, const char *_tag_name, const bool &_default );
     void getParameterBoolean ( bool &_value, const char *_tag_name );
     physics::JointPtr getJoint ( physics::ModelPtr &_parent, const char *_tag_name, const std::string &_joint_default_name );
     void isInitialized();
 
 
     template <class T>
     void getParameter ( T &_value, const char *_tag_name, const T &_default ) {
         _value = _default;
         if ( !sdf_->HasElement ( _tag_name ) ) {
             ROS_WARN_NAMED("utils", "%s: missing <%s> default is %s", info(), _tag_name, boost::lexical_cast<std::string> ( _default ).c_str() );
         } else {
             this->getParameter<T> ( _value, _tag_name );
         }
     }
     template <class T>
     void getParameter ( T &_value, const char *_tag_name ) {
         if ( sdf_->HasElement ( _tag_name ) ) {
             _value = sdf_->GetElement ( _tag_name )->Get<T>();
         }
         ROS_DEBUG_NAMED("utils", "%s: <%s> = %s", info(), _tag_name, boost::lexical_cast<std::string> ( _value ).c_str() );
 
     }
 
     template <class T>
     void getParameter ( T &_value, const char *_tag_name, const std::map<std::string, T> &_options, const T &_default ) {
         _value = _default;
         if ( !sdf_->HasElement ( _tag_name ) ) {
             ROS_WARN_NAMED("utils", "%s: missing <%s> default is %s", info(), _tag_name, boost::lexical_cast<std::string> ( _default ).c_str() );
         } else {
             this->getParameter<T> ( _value, _tag_name, _options );
         }
     }
     template <class T>
     void getParameter ( T &_value, const char *_tag_name, const std::map<std::string, T> &_options ) {
         typename std::map<std::string, T >::const_iterator it;
         if ( sdf_->HasElement ( _tag_name ) ) {
             std::string value = sdf_->GetElement ( _tag_name )->Get<std::string>();
             it = _options.find ( value );
             if ( it == _options.end() ) {
                 ROS_WARN_NAMED("utils", "%s: <%s> no matching key to %s", info(), _tag_name, value.c_str() );
             } else {
                 _value = it->second;
             }
         }
         ROS_DEBUG_NAMED("utils", "%s: <%s> = %s := %s",  info(), _tag_name, ( it == _options.end() ?"default":it->first.c_str() ), boost::lexical_cast<std::string> ( _value ).c_str() );
     }
 };
 
 typedef boost::shared_ptr<GazeboRos> GazeboRosPtr;
 }
 #endif
