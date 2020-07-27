# gazebo_tf_pub

This is a plugin that publishes the tfs using the gazebo world pose of a specific link declared into the urdf used by Gazebo.
Calling a specific service and declaring the link, if it exists,the plugin will publish the tf using like parent the "gazebo/world".
Furthermore, It's possible to publish all links always calling the same service inserting the keyword "all". The plugin will publish the tf of every link connect with its parent until the main "gazebo/world" tf is reached.
## How to run

1) Adding the plugin libgazebo_tf_plugin.so "Gazebo Plugin area" of the specific robot:

i.e: MultiDoF-superbuild/robots/centauro-simulator/centauro_gazebo/urdf/centauro.gazebo
    
     ......
     <xacro:if value="${middleware == 'xbotcore'}">
       <gazebo>
            <plugin name="gazebo_tf_plugins"  filename="CATKINK_PATH/devel/lib/libgazebo_tf_plugin.so"></plugin>
            <plugin name="gazebo_xbot_plugin" filename="libGazeboXBotPlugin.so">
                <path_to_config_file>${xbot_config_file}</path_to_config_file>
            </plugin>
      ......
2) Run Gazebo and play run.
3) rosservice call /gazebo_tfix_pub/gazebo/pub_tf_link "link_name: 'SPECIFI_LINK'" or insert 'all'.
4) Check the result on rviz for instance.
      
