final: prev:
{
  ros-gz-example-application = final.callPackage ./ros_gz_example_application/package.nix {};
  ros-gz-example-bringup = final.callPackage ./ros_gz_example_bringup/package.nix {};
  ros-gz-example-description = final.callPackage ./ros_gz_example_description/package.nix {};
  ros-gz-example-gazebo = final.callPackage ./ros_gz_example_gazebo/package.nix {};
  ros-gz-go-application = final.callPackage ./ros_gz_go_application/package.nix {};
}
