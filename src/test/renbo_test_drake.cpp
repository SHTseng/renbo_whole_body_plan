#include <ros/ros.h>
#include <ros/package.h>

#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/parsers/parser_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/material_map.h"

void ParseMaterial(tinyxml2::XMLElement* node, MaterialMap& materials);

void AddMaterialToMaterialMap(const std::string& material_name,
                              const Eigen::Vector4d& color_rgba,
                              bool abort_if_name_clash,
                              MaterialMap* materials);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "renbo_test_drake");
  ros::NodeHandle nh_;

  std::string package_path_ = ros::package::getPath("renbo_description");
  std::string urdf_path_ = package_path_.append("/robots/renbo_root_rfoot.urdf");

  std::string full_file_path = drake::parsers::GetFullPath(urdf_path_);
  ROS_INFO_STREAM(full_file_path);

  drake::parsers::PackageMap package_map_;
  package_map_.PopulateUpstreamToDrake(full_file_path);

  ros::Time begin = ros::Time::now();
  tinyxml2::XMLDocument urdf_xml_;
  urdf_xml_.LoadFile(full_file_path.data());
  if (urdf_xml_.ErrorID())
  {
    throw std::runtime_error("fail to parse xml in file" + full_file_path + urdf_xml_.ErrorName());
  }

  ros::Duration proc_time = ros::Time::now() - begin;
  ROS_INFO_STREAM("load urdf time: " << proc_time.toNSec()*1e-6 << " ms");

  tinyxml2::XMLElement* node = urdf_xml_.FirstChildElement("robot");
  if (!node)
  {
    throw std::runtime_error("urdf file does not contain robot tag.");
  }

  RigidBodyTree<double>* tree;
  const std::string model_name_ = node->Attribute("name");
  int model_instance_id = tree->add_model_instance();

  MaterialMap materials;
  for (tinyxml2::XMLElement* material_node = node->FirstChildElement("material");
       material_node;
       material_node = material_node->NextSiblingElement("material"))
  {
    ParseMaterial(material_node, materials);
  }

  ros::shutdown();

  return 0;
}

void ParseMaterial(tinyxml2::XMLElement* node, MaterialMap& materials) {
  const char* attr;
  attr = node->Attribute("name");
  if (!attr || strlen(attr) == 0) {
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