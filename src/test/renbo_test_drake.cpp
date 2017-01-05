#include <ros/ros.h>
#include <ros/package.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/parsers/parser_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/material_map.h"
#include "drake/util/drakeGeometryUtil.h"

#include "drake/thirdParty/bsd/spruce/spruce.hh"

void ParseMaterial(tinyxml2::XMLElement* node, MaterialMap& materials);

void AddMaterialToMaterialMap(const std::string& material_name,
                              const Eigen::Vector4d& color_rgba,
                              bool abort_if_name_clash,
                              MaterialMap* materials);

bool ParseBody(RigidBodyTree<double>* tree, std::string robot_name, tinyxml2::XMLElement* node,
               MaterialMap* materials, const drake::parsers::PackageMap& package_map,
               const std::string& root_dir, int model_instance_id, int* index);

void ParseVisual(RigidBody<double>* body, tinyxml2::XMLElement* node,
                 RigidBodyTree<double>* tree,
                 MaterialMap* materials, const drake::parsers::PackageMap& package_map,
                 const std::string& root_dir);

void ParseCollision(RigidBody<double>* body, tinyxml2::XMLElement* node,
                    RigidBodyTree<double>* tree,
                    const drake::parsers::PackageMap& package_map, const std::string& root_dir);

bool ParseGeometry(tinyxml2::XMLElement* node, const drake::parsers::PackageMap& package_map,
                   const std::string& root_dir,
        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                   DrakeShapes::Element& element);

void ParseInertial(RigidBody<double>* body, tinyxml2::XMLElement* node);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "renbo_test_drake");
  ros::NodeHandle nh_;

  std::string package_path_ = ros::package::getPath("renbo_description");
  std::string urdf_path_ = package_path_.append("/robots/renbo_root_rfoot.urdf");

//  std::string full_file_path = drake::parsers::GetFullPath(urdf_path_);
  std::string full_file_path = drake::GetDrakePath() + "/examples/RENBO/urdf/urdf/renbo_root_rfoot.urdf";
  ROS_INFO_STREAM(full_file_path);

  drake::parsers::PackageMap package_map_;
  package_map_.Add("RENBO", drake::GetDrakePath() + "/examples/RENBO");
  //package_map_.PopulateUpstreamToDrake(full_file_path);
  ROS_INFO_STREAM("read package num: " << package_map_.size());

  ros::Time begin = ros::Time::now();
  tinyxml2::XMLDocument urdf_xml_;
  urdf_xml_.LoadFile(full_file_path.data());
  if (urdf_xml_.ErrorID())
  {
    throw std::runtime_error("fail to parse xml in file" + full_file_path + urdf_xml_.ErrorName());
  }

  ros::Duration proc_time = ros::Time::now() - begin;
  ROS_INFO_STREAM("load urdf time: " << proc_time.toNSec()*1e-6 << " ms");

  std::string root_dir = ".";
  size_t found = full_file_path.find_last_of("/\\");
  if (found != std::string::npos)
  {
    root_dir = full_file_path.substr(0, found);
  }

  ROS_INFO_STREAM("read root: " << root_dir);

  tinyxml2::XMLElement* node = urdf_xml_.FirstChildElement("robot");
  if (!node)
  {
    throw std::runtime_error("urdf file does not contain robot tag.");
  }

  if (!node->Attribute("name"))
    throw std::runtime_error("Error: your robot must have a name attribute");

  auto tree = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::ModelInstanceIdTable model_instance_id_table_;
  std::string model_name_ = node->Attribute("name");
  int model_instance_id = tree.get()->add_model_instance();
  model_instance_id_table_[model_name_] = model_instance_id;

  MaterialMap materials;
  for (tinyxml2::XMLElement* material_node = node->FirstChildElement("material");
       material_node;
       material_node = material_node->NextSiblingElement("material"))
  {
    ParseMaterial(material_node, materials);
  }

  // in AddModelInstanceFromUrdfFileToWorld function, parse nullptr
  std::shared_ptr<RigidBodyFrame<double>> weld_to_frame = nullptr;

  // Makes a copy of parameter floating_base_type. This is necessary since the
  // actual type may be specified by the URDF itself when the URDF contains a
  // world link and a joint connecting to the world link. By default,
  // actual_floating_base_type equals parameter floating_base_type.
  drake::multibody::joints::FloatingBaseType actual_floating_base_type = drake::multibody::joints::kRollPitchYaw;

  // Maintains a list of links that were added to the rigid body tree.
  // This is iterated over by AddFloatingJoint() to determine where to attach
  // floating joints.
  std::vector<int> link_indices;

  // Parses the model's link elements.
  for (tinyxml2::XMLElement* link_node = node->FirstChildElement("link"); link_node;
       link_node = link_node->NextSiblingElement("link"))
  {
    int index;
    if (ParseBody(tree.get(), model_name_, link_node, &materials, package_map_, root_dir, model_instance_id, &index))
    {
      link_indices.push_back(index);
    }
    else
    {
      // Determines whether the link was not parsed because it is a world link.
      const char* name_attr = link_node->Attribute("name");
      if (!name_attr)
        throw std::runtime_error("ERROR: link tag is missing name attribute");

//      if (std::string(name_attr) == std::string(RigidBodyTree<double>::kWorldName)) {
//        // Since a world link was specified within the URDF, there must be
//        // a  joint that connects the world to the robot's root node. The
//        // following method parses the information contained within this
//        // joint. The weld_to_frame transform, if not a nullptr, is updated
//        // with the transform specified by this joint. If it is a nullptr,
//        // a shared_ptr<RigidBodyFrame> is created and initialized to this
//        // transform.
//        ParseWorldJoint(node, actual_floating_base_type, weld_to_frame);
      }
    }
//  }
//
//  // DEBUG
//  // else {
//  // cout << "Parsed link" << endl;
//  // cout << "model->bodies.size() = " << model->bodies.size() << endl;
//  // cout << "model->num_bodies = " << model->num_bodies << endl;
//  //}
//  // END_DEBUG
//
//  // todo: parse collision filter groups
//
//  // Parses the model's joint elements.
//  for (tinyxml2::XMLElement* joint_node = node->FirstChildElement("joint"); joint_node;
//       joint_node = joint_node->NextSiblingElement("joint"))
//    ParseJoint(tree, joint_node, model_instance_id);
//
//  // Parses the model's transmission elements.
//  for (tinyxml2::XMLElement* transmission_node = node->FirstChildElement("transmission");
//       transmission_node;
//       transmission_node =
//               transmission_node->NextSiblingElement("transmission"))
//    ParseTransmission(tree, transmission_node, model_instance_id);
//
//  // Parses the model's loop joint elements.
//  for (tinyxml2::XMLElement* loop_node = node->FirstChildElement("loop_joint"); loop_node;
//       loop_node = loop_node->NextSiblingElement("loop_joint"))
//    ParseLoop(tree, loop_node, model_instance_id);
//
//  // Parses the model's Drake frame elements.
//  for (tinyxml2::XMLElement* frame_node = node->FirstChildElement("frame"); frame_node;
//       frame_node = frame_node->NextSiblingElement("frame"))
//    ParseFrame(tree, frame_node, model_instance_id);
//
//  // Adds the floating joint(s) that connect the newly added robot model to the
//  // rest of the rigid body tree.
//  AddFloatingJoint(actual_floating_base_type, link_indices, weld_to_frame,
//                   nullptr /* pose_map */, tree);

  ROS_INFO_STREAM("finish test");

  ros::shutdown();

  return 0;
}

void ParseMaterial(tinyxml2::XMLElement* node, MaterialMap& materials)
{
  const char* attr;
  attr = node->Attribute("name");
  if (!attr || strlen(attr) == 0)
  {
    throw std::runtime_error(
            "RigidBodyTreeURDF.cpp: ParseMaterial(): ERROR: "
                    "Material tag is missing a name.");
  }
  std::string name(attr);

  Eigen::Vector4d rgba = Eigen::Vector4d::Zero();  // Defaults to black.

  tinyxml2::XMLElement* color_node = node->FirstChildElement("color");

  if (color_node) {
    if (!parseVectorAttribute(color_node, "rgba", rgba)) {
      throw std::runtime_error(
              "RigidBodyTreeURDF.cpp: ParseMaterial(): ERROR: "
                      "Color tag is missing rgba attribute.");
    }
    AddMaterialToMaterialMap(name, rgba, true /* abort_if_name_clash */,
                             &materials);
  } else {
    // If no color was specified and the material is not in the materials map,
    // check if the material is texture-based. If it is, print a warning, use
    // default color (black), and then return.
    //
    // Otherwise, throw an exception.
    //
    // TODO(liang.fok): Update this logic once texture-based materials are
    // supported. See: https://github.com/RobotLocomotion/drake/issues/2588.
    if (materials.find(name) == materials.end()) {
      tinyxml2::XMLElement* texture_node = node->FirstChildElement("texture");

      if (texture_node) {
        std::cerr
                << "RigidBodyTreeURDF.cpp: ParseMaterial():  WARNING: Material \""
                << name << "\" is a texture. Textures are currently not supported. "
                << "For more information, see: "
                << "https://github.com/RobotLocomotion/drake/issues/2588. "
                        "Defaulting to use the black color for this material."
                << std::endl;
        AddMaterialToMaterialMap(name, rgba, true /* abort_if_name_clash */,
                                 &materials);
      } else {
        throw std::runtime_error(
                "RigidBodyTreeURDF.cpp: ParseMaterial: ERROR: Material\"" + name +
                "\" not previously defined. Therefore a color must be specified.");
      }

      return;
    }
  }
}

void AddMaterialToMaterialMap(const std::string& material_name,
                              const Eigen::Vector4d& color_rgba,
                              bool abort_if_name_clash,
                              MaterialMap* materials) {
  // Verifies that parameter materials is not nullptr.
  DRAKE_DEMAND(materials);

  // Determines if the material is already in the map.
  auto material_iter = materials->find(material_name);
  if (material_iter != materials->end()) {
    // The material is already in the map. Checks whether the old material is
    // the same as the new material.  The range of values in the RGBA vectors
    // is [0, 1].
    const auto& existing_color = material_iter->second;
    if (abort_if_name_clash || (color_rgba != existing_color)) {
      // The materials map already has the material_name key but the color
      // associated with it is different.
      std::stringstream error_buff;
      error_buff << "Material \"" + material_name + "\" was previously "
                 << "defined." << std::endl
                 << "  - existing RGBA values: " << existing_color.transpose()
                 << std::endl
                 << "  - new RGBA values: " << color_rgba.transpose()
                 << std::endl;
      DRAKE_ABORT_MSG(error_buff.str().c_str());
    }
  } else {
    // Adds the new color to the materials map.
    (*materials)[material_name] = color_rgba;
  }
}

bool ParseBody(RigidBodyTree<double>* tree, std::string robot_name, tinyxml2::XMLElement* node,
               MaterialMap* materials, const drake::parsers::PackageMap& package_map,
               const std::string& root_dir, int model_instance_id, int* index)
{
  const char* attr = node->Attribute("drake_ignore");
  if (attr && (std::strcmp(attr, "true") == 0)) return false;

  RigidBody<double>* body{nullptr};
  std::unique_ptr<RigidBody<double>> owned_body(body = new RigidBody<double>());
  body->set_model_name(robot_name);
  body->set_model_instance_id(model_instance_id);

  attr = node->Attribute("name");
  if (!attr) throw std::runtime_error("ERROR: link tag is missing name attribute");

  // World links are handled by ParseWorldJoint().
  body->set_name(attr);
  if (body->get_name() == std::string(RigidBodyTree<double>::kWorldName))
    return false;

  tinyxml2::XMLElement* inertial_node = node->FirstChildElement("inertial");
  if (inertial_node) ParseInertial(body, inertial_node);

  for (tinyxml2::XMLElement* visual_node = node->FirstChildElement("visual"); visual_node;
       visual_node = visual_node->NextSiblingElement("visual")) {
    ParseVisual(body, visual_node, tree, materials, package_map, root_dir);
  }

  for (tinyxml2::XMLElement* collision_node = node->FirstChildElement("collision");
       collision_node;
       collision_node = collision_node->NextSiblingElement("collision")) {
    ParseCollision(body, collision_node, tree, package_map, root_dir);
  }

  tree->add_rigid_body(std::move(owned_body));
  *index = body->get_body_index();
  return true;
}

void ParseVisual(RigidBody<double>* body, tinyxml2::XMLElement* node,
                 RigidBodyTree<double>* tree,
                 MaterialMap* materials, const drake::parsers::PackageMap& package_map,
                 const std::string& root_dir) {
  // Ensures there is a geometry child element. Since this is a required
  // element, throws an exception if a geometry element does not exist.
  tinyxml2::XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node)
  {
    throw std::runtime_error("ERROR: Link " + body->get_name() +
                        " has a visual element without geometry.");
  }

  // Obtains the reference frame of the visualization relative to the reference
  // frame of the rigid body that is being visualized. It defaults to identity
  // if no transform is specified.
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  {
    tinyxml2::XMLElement* origin = node->FirstChildElement("origin");
    if (origin) originAttributesToTransform(origin, T_element_to_link);
  }
  DrakeShapes::VisualElement element(T_element_to_link);

  // Parses the geometry specifications of the visualization.
  if (!ParseGeometry(geometry_node, package_map, root_dir, element))
    throw std::runtime_error("ERROR: Failed to parse visual element in link " +
                        body->get_name() + ".");

  // Parses the material specification of the visualization. Note that we cannot
  // reuse the logic within ParseMaterial() here because the context is
  // different. Whereas ParseMaterial() parses material specifications that
  // are children elements of the "robot" element, the material specification
  // being parsed here are children of a "visual" element. One key difference is
  // the XML here may not specify a "name" attribute. Because of this difference
  // in context, we need specialized logic here to determine the material
  // visualization of a link.
  tinyxml2::XMLElement* material_node = node->FirstChildElement("material");
  if (material_node) {
    // Checks and remembers whether a "color" child element exists. If so,
    // parses the color value.
    bool color_specified = false;
    Eigen::Vector4d rgba;
    {
      tinyxml2::XMLElement* color_node = material_node->FirstChildElement("color");
      if (color_node) {
        if (!parseVectorAttribute(color_node, "rgba", rgba)) {
          throw std::runtime_error(
                  "ERROR: Failed to parse color of material for "
                          "model \"" +
                  body->get_model_name() + "\", link \"" + body->get_name() +
                  "\".");
        }
        color_specified = true;
      }
    }

    // Checks and remembers whether a "name" attribute exists. If so, parses the
    // name value.
    std::string material_name;
    bool name_specified = false;
    {
      const char* attr = material_node->Attribute("name");

      if (attr != nullptr && strlen(attr) != 0) {
        material_name = std::string(attr);
        name_specified = true;
      }
    }

    // Adds the material to the materials map if both the name and color are
    // specified. This is so that link elements that reside later in the URDF
    // can reference this material in their visualization elements. Note that
    // this capability is not specified by the official URDF specification (see:
    // http://wiki.ros.org/urdf/XML/link), but is needed by certain URDFs
    // released by companies and organizations like Robotiq and ROS Industrial
    // (for example, see this URDF by Robotiq: http://bit.ly/28P0pmo).
    if (color_specified && name_specified) {
      // The `abort_if_name_clash` parameter is passed a value of `false` to
      // allow the same material to be defined across multiple links as long as
      // they correspond to the same RGBA value. This can happen, for example,
      // in URDFs that are automatically generated using `xacro` since `xacro`
      // may produce a URDF from multiple `.xacro` files. Through testing, we
      // determined that the Gazebo simulator supports loading URDFs containing
      // duplicate material specifications as long as the duplicates are
      // distributed across multiple `<link>` elements and are not at the
      // `<robot>` level.
      AddMaterialToMaterialMap(material_name, rgba,
                               false /* abort_if_name_clash */, materials);
    }

    // Sets the material's color.
    bool material_set = false;
    {
      // If the color is specified as a child element of the current material
      // node, use that color. It takes precedence over any material saved in
      // the material map.
      if (color_specified) {
        element.setMaterial(rgba);
        material_set = true;
      } else {
        // No color specified. Checks if the material is already in the
        // materials map.
        if (name_specified) {
          auto material_iter = materials->find(material_name);
          if (material_iter != materials->end()) {
            // The material is in the map. Sets the material of the visual
            // element based on the value in the map.
            element.setMaterial(material_iter->second);
            material_set = true;
          }
        }
      }
    }

    // Throws a std::runtime_error if the material was not set for this
    // visualization.
    //
    // TODO(liang.fok): Update this logic once texture-based materials are
    // supported. See: https://github.com/RobotLocomotion/drake/issues/2588.
    if (!material_set) {
      std::stringstream error_buff;
      error_buff
              << "RigidBodyTreeURDF.cpp: ParseVisual(): "
              << "WARNING: Visual element has a material whose color could not"
                      "be determined."
              << std::endl
              << "  - model name: " << body->get_model_name() << std::endl
              << "  - body name: " << body->get_name() << std::endl
              << "  - material name: " << material_name << std::endl;
      throw std::runtime_error(error_buff.str());
    }
  }

  if (element.hasGeometry()) body->AddVisualElement(element);
}

void ParseCollision(RigidBody<double>* body, tinyxml2::XMLElement* node,
                    RigidBodyTree<double>* tree,
                    const drake::parsers::PackageMap& package_map, const std::string& root_dir) {
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  tinyxml2::XMLElement* origin = node->FirstChildElement("origin");
  if (origin) originAttributesToTransform(origin, T_element_to_link);

  const char* attr;
  std::string group_name;

  attr = node->Attribute("group");
  if (attr) {
    group_name = attr;
  } else {
    group_name = "default";
  }

  tinyxml2::XMLElement* geometry_node = node->FirstChildElement("geometry");
  if (!geometry_node)
    throw std::runtime_error("ERROR: Link " + body->get_name() +
                        " has a collision element without geometry");

  DrakeCollision::Element element(T_element_to_link, body);
  // By default all collision elements added to the world from an URDF file are
  // flagged as static.
  // We would also like to flag as static bodies connected to the world with a
  // FloatingBaseType::kFixed joint.
  // However this is not possible at this stage since joints were not parsed
  // yet.
  // Solutions to this problem would be:
  //  1. To load the model with DrakeCollision::Element's here but flag them as
  //     static later at a the compile stage. This means that Bullet objects are
  //     not created here (with addCollisionElement) but later on with the call
  //     to RBT::compile when all the connectivity information is available.
  //  2. Load collision elements on a separate pass after links and joints were
  //     already loaded.
  //  Issue 2661 was created to track this problem.
  // TODO(amcastro-tri): fix the above issue tracked by 2661.  Similarly for
  // parseSDFCollision in RigidBodyTreeSDF.cpp.
  if (body->get_name().compare(std::string(RigidBodyTree<double>::kWorldName)) == 0)
    element.set_static();
  if (!ParseGeometry(geometry_node, package_map, root_dir, element))
    throw std::runtime_error("ERROR: Failed to parse collision element in link " +
                        body->get_name() + ".");

  if (element.hasGeometry()) {
    tree->addCollisionElement(element, *body, group_name);
  }
}

bool ParseGeometry(tinyxml2::XMLElement* node, const drake::parsers::PackageMap& package_map,
                   const std::string& root_dir,
        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                   DrakeShapes::Element& element) {
  // DEBUG
  // cout << "ParseGeometry: START" << endl;
  // END_DEBUG
  const char* attr;
  tinyxml2::XMLElement* shape_node;
  if ((shape_node = node->FirstChildElement("box"))) {
    double x = 0, y = 0, z = 0;
    attr = shape_node->Attribute("size");
    if (attr) {
      std::stringstream s(attr);
      s >> x >> y >> z;
    } else {
      std::cerr << "ERROR parsing box element size" << std::endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Box(Eigen::Vector3d(x, y, z)));
  } else if ((shape_node = node->FirstChildElement("sphere"))) {
    double r = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      std::stringstream s(attr);
      s >> r;
    } else {
      std::cerr << "ERROR parsing sphere element radius" << std::endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Sphere(std::max(DrakeShapes::MIN_RADIUS, r)));
  } else if ((shape_node = node->FirstChildElement("cylinder"))) {
    double r = 0, l = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      std::stringstream s(attr);
      s >> r;
    } else {
      std::cerr << "ERROR parsing cylinder element radius" << std::endl;
      return false;
    }

    attr = shape_node->Attribute("length");
    if (attr) {
      std::stringstream s(attr);
      s >> l;
    } else {
      std::cerr << "ERROR parsing cylinder element length" << std::endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Cylinder(r, l));
  } else if ((shape_node = node->FirstChildElement("capsule"))) {
    double r = 0, l = 0;
    attr = shape_node->Attribute("radius");
    if (attr) {
      std::stringstream s(attr);
      s >> r;
    } else {
      std::cerr << "ERROR parsing capsule element radius" << std::endl;
      return false;
    }

    attr = shape_node->Attribute("length");
    if (attr) {
      std::stringstream s(attr);
      s >> l;
    } else {
      std::cerr << "ERROR: Failed to parse capsule element length" << std::endl;
      return false;
    }
    element.setGeometry(DrakeShapes::Capsule(r, l));
  }
  else if ((shape_node = node->FirstChildElement("mesh")))
  {
    attr = shape_node->Attribute("filename");
    if (!attr) {
      std::cerr << "ERROR mesh element has no filename tag" << std::endl;
      return false;
    }
    std::string filename(attr);

    spruce::path mesh_filename_spruce;
    spruce::path raw_filename_spruce(filename);

    std::vector<std::string> split_filename = raw_filename_spruce.split();
    ROS_INFO_STREAM(filename);
    ROS_INFO_STREAM(split_filename[2]);

    std::string resolved_filename;
    // This method will return an empty string if the file is not found or
    // resolved within a ROS package.
    resolved_filename = ResolveFilename(filename, package_map, root_dir);
    ROS_INFO_STREAM("resolved file name: " << resolved_filename);
//    if (package_map.Contains(split_filename[2]))
//    {
//      std::string package_path_string = package_map.GetPath(split_filename[2]);
//      mesh_filename_spruce = spruce::path(package_path_string);
//    }

    if (resolved_filename.empty())
    {
      throw std::runtime_error(
              std::string(__FILE__) + ": " + __func__ +
              ": ERROR: Mesh file name could not be resolved from the "
                      "provided uri \"" +
              filename + "\".");
    }
    DrakeShapes::Mesh mesh(filename, resolved_filename);

    // Obtains the scale of the mesh if it exists.
    if (shape_node->Attribute("scale") != nullptr)
      ParseThreeVectorAttribute(shape_node, "scale", &mesh.scale_);

    element.setGeometry(mesh);
  } else {
    std::cerr << "Warning: geometry element has an unknown type and will be ignored."
         << std::endl;
  }
  // DEBUG
  // cout << "ParseGeometry: END" << endl;
  // END_DEBUG
  return true;
}

void ParseInertial(RigidBody<double>* body, tinyxml2::XMLElement* node) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

  tinyxml2::XMLElement* origin = node->FirstChildElement("origin");
  if (origin) originAttributesToTransform(origin, T);

  tinyxml2::XMLElement* mass = node->FirstChildElement("mass");
  if (mass) {
    double body_mass = 0;
    parseScalarAttribute(mass, "value", body_mass);
    body->set_mass(body_mass);
  }

  Eigen::Vector3d com;
  com << T(0, 3), T(1, 3), T(2, 3);
  body->set_center_of_mass(com);

  drake::SquareTwistMatrix<double> I = drake::SquareTwistMatrix<double>::Zero();
  I.block(3, 3, 3, 3) << body->get_mass() * Eigen::Matrix3d::Identity();

  tinyxml2::XMLElement* inertia = node->FirstChildElement("inertia");
  if (inertia) {
    parseScalarAttribute(inertia, "ixx", I(0, 0));
    parseScalarAttribute(inertia, "ixy", I(0, 1));
    I(1, 0) = I(0, 1);
    parseScalarAttribute(inertia, "ixz", I(0, 2));
    I(2, 0) = I(0, 2);
    parseScalarAttribute(inertia, "iyy", I(1, 1));
    parseScalarAttribute(inertia, "iyz", I(1, 2));
    I(2, 1) = I(1, 2);
    parseScalarAttribute(inertia, "izz", I(2, 2));
  }

  body->set_spatial_inertia(transformSpatialInertia(T, I));
}