# gazebo_tf_pub

This is a plugin that publishes the tfs using the gazebo world pose of a specific link declared into the urdf used by Gazebo.
Calling a specific service and declaring the link, if it exists,the plugin will publish the tf using like parent the "gazebo/world".
Furthermore, It's possible to publish all links always calling the same service inserting the keyword "all". The plugin will publish the tf of every link connect with its parent until the main "gazebo/world" tf is reached.
