add_library_with_exports(LIB_NAME drakeXMLUtil SOURCE_FILES xml_util.cc)
target_link_libraries(drakeXMLUtil drakeCommon tinyxml2 spruce)
drake_install_libraries(drakeXMLUtil)
drake_install_headers(xml_util.h)
drake_install_pkg_config_file(drake-xml-util
  LIBS -ldrakeXMLUtil -ltinyxml2 -lspruce
  REQUIRES
    drake-common
    eigen3)

add_library_with_exports(LIB_NAME drakeRBM SOURCE_FILES
  kinematics_cache.cc
  package_map.cc
  parser_model_instance_id_table.cc
  parser_common.cc
  parser_sdf.cc
  parser_urdf.cc
  rigid_body_tree.cc
  rigid_body.cc
  rigid_body_tree_contact.cc
  rigid_body_actuator.cc
  rigid_body_frame.cc
  rigid_body_loop.cc
  rigid_body_tree_construction.cc
  $<TARGET_OBJECTS:drakeUtil>)
target_link_libraries(drakeRBM
  drakeCollision drakeCommon drakeJoints drakeXMLUtil)
drake_install_libraries(drakeRBM)
drake_install_headers(
  constraint_wrappers.h
  force_torque_measurement.h
  kinematic_path.h
  kinematics_cache.h
  kinematics_cache-inl.h
  material_map.h
  package_map.h
  parser_common.h
  parser_model_instance_id_table.h
  parser_sdf.h
  parser_urdf.h
  pose_map.h
  rigid_body.h
  rigid_body_actuator.h
  rigid_body_frame.h
  rigid_body_loop.h
  rigid_body_tree.h
  rigid_body_tree_construction.h)

drake_install_pkg_config_file(drake-rbm
  TARGET drakeRBM
  LIBS -ldrakeRBM
  REQUIRES
    drake-collision
    drake-common
    drake-joints
    drake-xml-util
    eigen3)

add_library_with_exports(LIB_NAME drakeIKoptions SOURCE_FILES ik_options.cc)
target_link_libraries(drakeIKoptions drakeRBM)
drake_install_pkg_config_file(drake-ik-options
  TARGET drakeIKoptions
  LIBS -ldrakeIKoptions
  REQUIRES
    drake-rbm
    eigen3)

set(drakeIK_SRC_FILES constraint_wrappers.cc ik_trajectory_helper.cc
  inverse_kinematics.cc inverse_kinematics_backend.cc inverse_kinematics_pointwise.cc
  inverse_kinematics_trajectory.cc inverse_kinematics_trajectory_backend.cc)
if (gurobi_FOUND)
  set(drakeIK_SRC_FILES ${drakeIK_SRC_FILES} approximate_ik.cc)
endif ()
add_library_with_exports(LIB_NAME drakeIK SOURCE_FILES ${drakeIK_SRC_FILES})
target_link_libraries(drakeIK drakeRBM drakeOptimization drakeRigidBodyConstraint drakeIKoptions)
drake_install_libraries(drakeIK)
drake_install_libraries(drakeIKoptions)
drake_install_headers(constraint_wrappers.h ik_trajectory_helper.h ik_options.h
  inverse_kinematics_backend.h rigid_body_ik.h)
drake_install_pkg_config_file(drake-ik
  TARGET drakeIK
  LIBS -ldrakeIK
  REQUIRES
    drake-ik-options
    drake-optimization
    drake-rbm
    drake-rigid-body-constraint
    eigen3)

add_subdirectory(collision)
add_subdirectory(constraint)
add_subdirectory(joints)
add_subdirectory(rigid_body_plant)
add_subdirectory(rigid_body_system1)
add_subdirectory(shapes)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
