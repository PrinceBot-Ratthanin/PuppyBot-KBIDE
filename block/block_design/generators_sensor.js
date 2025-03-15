module.exports = function(Blockly){
  'use strict';
Blockly.JavaScript['sw1_press'] = function(block) {  
  var code = 'wait_SW1();\n';
  return code;
};
Blockly.JavaScript['sw1_press_one'] = function(block) {  
  var code = 'wait_SW1_one();\n';
  return code;
};
Blockly.JavaScript['button_1_status'] = function(block) {  
  var code = '(digitalRead(6))';  
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};
Blockly.JavaScript['INPUT_digital'] = function(block) { 
  var value_pin = Blockly.JavaScript.valueToCode(block, 'pin', Blockly.JavaScript.ORDER_ATOMIC) || '0'; 
  var code = '(IN('+value_pin+'))';  
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};
Blockly.JavaScript['TCS_color_status'] = function(block) {
  var dropdown_pin = block.getFieldValue('_color');
  var code = `(Read_Color_TCS(${dropdown_pin}))`;
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};

Blockly.JavaScript['TCS_color_config'] = function(block) {
  var dropdown_pinSDA = block.getFieldValue('pin_SDA');
  var dropdown_pinSCL = block.getFieldValue('pin_SCL');
  var code = `
              #SETUP  Wire1.setSDA(${dropdown_pinSDA});Wire1.setSCL(${dropdown_pinSCL});if (tcsB.begin(0x29,&Wire1)) {Serial.println("Found sensor");} #END `;
              // (Read_Color_TCS(${dropdown_pin}))`;
  return code;
};

Blockly.JavaScript['TCS_color_status_B'] = function(block) {
  var dropdown_pin = block.getFieldValue('_color');
  var code = `#SETUP  Wire1.setSDA(14);Wire1.setSCL(15);if (tcsB.begin(0x29,&Wire1)) {Serial.println("Found sensor");} #END
              (Read_Color_TCS_B(${dropdown_pin}))`;
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};
Blockly.JavaScript['Light_Sensor'] = function(block) {
  var dropdown_pin = block.getFieldValue('pin');
  var code = `(ADC(${dropdown_pin}))`;
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};

Blockly.JavaScript['Ultrasonic_sensor'] = function(block) {  
  var dropdown_pin_Echo = block.getFieldValue('pin_Echo');
  var dropdown_pin_Trig = block.getFieldValue('pin_Trig');
  var code = `(ultrasonic(${dropdown_pin_Echo},${dropdown_pin_Trig}))`;
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};

Blockly.JavaScript['IMU_begin'] = function(block) {  

  var code = '#SETUP initIMU();\nupdateIMU_fine_offset_Yaw(200); #END\n';
  return code;
};
Blockly.JavaScript['IMU_update'] = function(block) {  
  var code = 'mpu6050.update();\n';
  return code;
};
Blockly.JavaScript['IMU_getData_Yaw'] = function(block) {
  var dropdown_axis = block.getFieldValue('axis');

  var code = ``;
  if(dropdown_axis == '0'){code += `getYaw()`}
    if(dropdown_axis == '1'){code += `getContinuousYaw()`}
      if(dropdown_axis == '2'){code += `getOffsetYaw()`}
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};
Blockly.JavaScript['IMU_getData'] = function(block) {
  var dropdown_axis = block.getFieldValue('axis');

  var code = ``;
  if(dropdown_axis == '0'){code += `getPitch()`}
    if(dropdown_axis == '1'){code += `getRoll()`}
      if(dropdown_axis == '2'){code += `getYaw()`}
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};
Blockly.JavaScript['IMU_TurnPID'] = function(block) {
  var value_s0 = Blockly.JavaScript.valueToCode(block, 'S0', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s1 = Blockly.JavaScript.valueToCode(block, 'S1', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s2 = Blockly.JavaScript.valueToCode(block, 'S2', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s3 = Blockly.JavaScript.valueToCode(block, 'S3', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s4 = Blockly.JavaScript.valueToCode(block, 'S4', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var code = '';
  //code += 'PID_NumPin = ' + value_numSensor+';\t';
  code += 'turnPID('+value_s0+','+value_s1+','+value_s2+','+value_s3+','+value_s4+');\n';
  return code;
};

Blockly.JavaScript['IMU_moveStraightPID'] = function(block) {
  var dropdown_dir = block.getFieldValue('dir');
  var value_s0 = Blockly.JavaScript.valueToCode(block, 'S0', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s1 = Blockly.JavaScript.valueToCode(block, 'S1', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s2 = Blockly.JavaScript.valueToCode(block, 'S2', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s3 = Blockly.JavaScript.valueToCode(block, 'S3', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s4 = Blockly.JavaScript.valueToCode(block, 'S4', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s5 = Blockly.JavaScript.valueToCode(block, 'S5', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var checkbox_newline = (block.getFieldValue('Slow') == 'TRUE')? '1' : '0';

  var code = '';
  //code += 'PID_NumPin = ' + value_numSensor+';\t';
  code += 'moveStraightPID('+dropdown_dir+','+value_s0+','+value_s1+','+value_s2+','+value_s3+','+value_s4+','+value_s5+','+checkbox_newline+');\n';
  return code;
};
Blockly.JavaScript['IMU_set_dataFor_turnDirection'] = function(block) {
  var value_s0 = Blockly.JavaScript.valueToCode(block, 'S0', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s1 = Blockly.JavaScript.valueToCode(block, 'S1', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s2 = Blockly.JavaScript.valueToCode(block, 'S2', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s3 = Blockly.JavaScript.valueToCode(block, 'S3', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var code = '';
  //code += 'PID_NumPin = ' + value_numSensor+';\t';
  code += 'set_data_for_turnDirection('+value_s0+','+value_s1+','+value_s2+','+value_s3+');\n';
  return code;
};

Blockly.JavaScript['IMU_TurnByDirection'] = function(block) {
  var dropdown_direction = block.getFieldValue('direction');

  var code = ``;
  if(dropdown_direction == '0'){code += `turnNorth();`}
  if(dropdown_direction == '1'){code += `turnEast();`}
  if(dropdown_direction == '2'){code += `turnSouth();`}
  if(dropdown_direction == '3'){code += `turnWest();`}
  return code;
};

Blockly.JavaScript['IMU_TurnByAngle'] = function(block) {
  var dropdown_angle = block.getFieldValue('angle');

  var code = ``;
  if(dropdown_angle == '0'){code += `turnByAngle(90);`}
  if(dropdown_angle == '1'){code += `turnByAngle(-90);`}
  if(dropdown_angle == '2'){code += `turnByAngle(180);`}
  if(dropdown_angle == '3'){code += `turnByAngle(-180);`}
  return code;
};


Blockly.JavaScript['IMU_moveStraightDirection'] = function(block) {
  var dropdown_direction = block.getFieldValue('dir');
  var dropdown_angle = block.getFieldValue('angle');
  
  var value_s1 = Blockly.JavaScript.valueToCode(block, 'S1', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s2 = Blockly.JavaScript.valueToCode(block, 'S2', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s3 = Blockly.JavaScript.valueToCode(block, 'S3', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s4 = Blockly.JavaScript.valueToCode(block, 'S4', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s5 = Blockly.JavaScript.valueToCode(block, 'S5', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var checkbox_newline = (block.getFieldValue('Slow') == 'TRUE')? '1' : '0';

  var code = ``;
  code += 'MoveStraightDirection('+dropdown_direction+','+dropdown_angle+','+value_s1+','+value_s2+','+value_s3+','+value_s4+','+value_s5+','+checkbox_newline+');\n';
  return code;
};

Blockly.JavaScript['IMU_moveStraightPID_Encoder'] = function(block) {
  var dropdown_dir = block.getFieldValue('dir');
  var value_s0 = Blockly.JavaScript.valueToCode(block, 'S0', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s1 = Blockly.JavaScript.valueToCode(block, 'S1', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s2 = Blockly.JavaScript.valueToCode(block, 'S2', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s3 = Blockly.JavaScript.valueToCode(block, 'S3', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s4 = Blockly.JavaScript.valueToCode(block, 'S4', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s5 = Blockly.JavaScript.valueToCode(block, 'S5', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var checkbox_newline = (block.getFieldValue('Slow') == 'TRUE')? '1' : '0';

  var code = '';
  //code += 'PID_NumPin = ' + value_numSensor+';\t';
  code += 'moveStraightPID_Encoder('+dropdown_dir+','+value_s0+','+value_s1+','+value_s2+','+value_s3+','+value_s4+','+value_s5+','+checkbox_newline+');\n';
  return code;
};

Blockly.JavaScript['IMU_moveStraightDirection_Encoder'] = function(block) {
  var dropdown_direction = block.getFieldValue('dir');
  var dropdown_angle = block.getFieldValue('angle');
  
  var value_s1 = Blockly.JavaScript.valueToCode(block, 'S1', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s2 = Blockly.JavaScript.valueToCode(block, 'S2', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s3 = Blockly.JavaScript.valueToCode(block, 'S3', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s4 = Blockly.JavaScript.valueToCode(block, 'S4', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var value_s5 = Blockly.JavaScript.valueToCode(block, 'S5', Blockly.JavaScript.ORDER_ATOMIC) || '0';
  var checkbox_newline = (block.getFieldValue('Slow') == 'TRUE')? '1' : '0';

  var code = ``;
  code += 'MoveStraightDirection_Encoder('+dropdown_direction+','+dropdown_angle+','+value_s1+','+value_s2+','+value_s3+','+value_s4+','+value_s5+','+checkbox_newline+');\n';
  return code;
};

Blockly.JavaScript['reset_Encoder'] = function(block) {  
  var code = '#SETUP init_encoder();\n#END\n resetEncoder();\n';
  return code;
};
Blockly.JavaScript['read_Encoder'] = function(block) {
 

  var code = `#SETUP init_encoder();\n#END\n get_pulse_Encoder()`;
  return [code, Blockly.JavaScript.ORDER_ATOMIC];
};


}