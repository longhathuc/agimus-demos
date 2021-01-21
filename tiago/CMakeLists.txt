# Install launch files in launch directory
INSTALL_DEMO_LAUNCH_FILES(insert_drill
  estimation.launch
  gazebo.launch
  geometric_simu.launch
  tiago.launch
  tiago_hpp.launch
  world_setup.launch
  )

INSTALL_DEMO_LAUNCH_FILES(follow_box
  estimation.launch
  gazebo.launch
  geometric_simu.launch
  tiago_sot.launch
  tiago_hpp.launch
  world_setup.launch
  tiago_controller.launch
  )

  INSTALL_DEMO_LAUNCH_FILES(deburr
  estimation.launch
  gazebo.launch
  geometric_simu.launch
  tiago_sot.launch
  tiago_hpp.launch
  world_setup.launch
  tiago_controller.launch
  )


  INSTALL_DEMO_LAUNCH_FILES(gerard_bauzil
  tiago_gazebo.launch
  )