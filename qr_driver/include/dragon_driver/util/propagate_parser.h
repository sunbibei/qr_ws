/*
 * propagate_parser.h
 *
 *  Created on: 2017年1月17日
 *      Author: zhangzhi
 */

#ifndef INCLUDE_DRAGON_DRIVER_UTIL_PROPAGATE_PARSER_H_
#define INCLUDE_DRAGON_DRIVER_UTIL_PROPAGATE_PARSER_H_

#define linux
#include <PCANBasic.h>
#include "../propagate/propagate.h"
#include "../hardware/motor.h"
#include "../hardware/encoder.h"

/*
#define KNEE "knee"
#define HIP "hip"
#define YAW "yaw"
#define LEFT_FRONT "left_front"
#define LEFT_BACK "left_back"
#define RIGHT_FRONT "right_front"
#define RIGHT_BACK "right_back"
*/

namespace middleware {

class PropagateParser {
public:
  PropagateParser();
  virtual ~PropagateParser();
  
  bool parsePcan(TPCANMsg& ,  Component<HwState>&);
  TPCANMsg packagePCAN(const std::string& , Component<HwCommand>&);
  std::string getJointName(TPCANMsg&); 
  std::string getDataType(TPCANMsg&); 

private:
  TPCANMsg msg_;
  short    position_;
  short    velocity_;
  short    ele_current_;
  double   last_position_;
  double   last_velocity_;
  double   last_ele_current_;
  short offset_;

  //const variable
  const std::string HIP = "hip";
  const std::string KNEE = "knee";
  const std::string YAW = "yaw";
  const std::string LEFT_FRONT = "left_front";
  const std::string LEFT_BACK = "left_back";
  const std::string RIGHT_FRONT = "right_front";
  const std::string RIGHT_BACK = "right_back";
  const std::string POSITION = "position";
  const std::string VELOCITY = "velocity";
  const std::string EFFORT = "effort";
  const std::string ELE_CURRENT = "ele_current";

 
  std::vector<std::string> names_;
  std::string              leg_name_;
  std::string              joint_name_;
  std::string              name_;
  std::string              dataType_ ;
};

} /* namespace middleware */

#endif /* INCLUDE_DRAGON_DRIVER_UTIL_PROPAGATE_PARSER_H_ */
