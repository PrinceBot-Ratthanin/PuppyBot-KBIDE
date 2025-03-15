const dirIcon = Vue.prototype.$global.board.board_info.dir;
module.exports = function(Blockly){
  'use strict';

Blockly.Blocks['sw1_press'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("SW1_press");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("Buzzer beep");
 this.setHelpUrl("");
  }
};
Blockly.Blocks['sw1_press_one'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("SW1_press one sensor ");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("Buzzer beep");
 this.setHelpUrl("");
  }
};
Blockly.Blocks['button_1_status'] = {
  init: function() {
    this.appendDummyInput()
        .appendField(new Blockly.FieldImage("/static/icons/sw12x12.png", 20, 20, "*"))
        .appendField("SW1(IO6) Status ");
    this.setInputsInline(true);
    this.setOutput(true, "Number");
    this.setColour(65);
 this.setTooltip("get SW1 pressed or not");
 this.setHelpUrl("");
  }
};

// Blockly.Blocks['Ultrasinoc_sensor'] = {
//   init: function() {
//     this.appendDummyInput()
//         //.appendField(new Blockly.FieldImage("/static/icons/bmx055.png", 20, 20, "*"))
//         .appendField("Read Distance");
//     this.setInputsInline(true);
//     this.setOutput(true, "Number");
//     this.setColour(65);
//  this.setTooltip("Read Distance");
//  this.setHelpUrl("");
//   }
// };
Blockly.Blocks['Light_Sensor'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Light sensor(ADC):")
        .appendField(new Blockly.FieldDropdown([["0","0"],["1","1"], ["2","2"], ["3","3"],
        										["4","4"],["5","5"], ["6","6"], ["7","7"], ["8","8"], ["9","9"], ["10","10"]]), "pin");
    this.setOutput(true);
    this.setColour(65);
    this.setTooltip("read pos EncoderA Pin 26");
    this.setHelpUrl("");
  }
};
Blockly.Blocks['INPUT_digital'] = {
  init: function() {
    // this.appendDummyInput()
    //     //.appendField(new Blockly.FieldImage("/static/icons/bmx055.png", 20, 20, "*"))
    //     .appendField("Read digital status");
    this.appendValueInput("pin")
      .appendField("Read digital status at");
    this.setInputsInline(true);
    this.setOutput(true, "Number");
    this.setColour(65);
 this.setTooltip("Read digital status at");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['Puppy_beep'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Buzzer beep");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("Buzzer beep");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['TCS_color_status'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("TCS_color at")
          .appendField(new Blockly.FieldDropdown([["R","0"],["G","1"], ["B","2"], ["colorTemp","3"], ["lux","4"]]), "_color");
      this.setInputsInline(true);
      this.setOutput(true, "Number");
      this.setColour(65);
   this.setTooltip("color");
   this.setHelpUrl("");
    }
  };

Blockly.Blocks['Ultrasonic_sensor'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Ulrasonic Config Pin Echo")
        .appendField(new Blockly.FieldDropdown([["S1","12"],["S2","13"], ["S3","14"], ["S4","15"],
                            ["D1","25"],["D2","27"], ["D3","28"], ["D4","29"]]), "pin_Echo");
    this.appendDummyInput()
        .appendField("Pin Trig")
        .appendField(new Blockly.FieldDropdown([["S2","13"],["S1","12"], ["S3","14"], ["S4","15"],
                            ["D1","25"],["D2","27"], ["D3","28"], ["D4","29"]]), "pin_Trig");
    this.setInputsInline(true);
    this.setOutput(true, "Number");
    this.setColour(65);
    this.setTooltip("config pin for Ultrasonic");
    this.setHelpUrl("");
  }
};

Blockly.Blocks['TCS_color_status_B'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("TCS_color_2 at")
          .appendField(new Blockly.FieldDropdown([["R","0"],["G","1"], ["B","2"], ["colorTemp","3"], ["lux","4"]]), "_color");
      this.setInputsInline(true);
      this.setOutput(true, "Number");
      this.setColour(65);
   this.setTooltip("color");
   this.setHelpUrl("");
    }
  };
Blockly.Blocks['IMU_begin'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("IMU begin and calcGyroOffsets");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("IMU Init");
 this.setHelpUrl("");
  }
};
Blockly.Blocks['IMU_update'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Gyro update");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("IMU update");
 this.setHelpUrl("");
  }
};
Blockly.Blocks['IMU_getData_Yaw'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("getYawData")
          .appendField(new Blockly.FieldDropdown([["getCurrentYaw","0"],["getContinuousYaw","1"], ["getOffsetYaw","2"]]), "axis");
      this.setInputsInline(true);
      this.setOutput(true, "Number");
      this.setColour(65);
   this.setTooltip("Get Data from IMU");
   this.setHelpUrl("");
    }
  };
Blockly.Blocks['IMU_getData'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("getData")
          .appendField(new Blockly.FieldDropdown([["getPitch","0"],["getRoll","1"], ["getYaw","2"]]), "axis");
      this.setInputsInline(true);
      this.setOutput(true, "Number");
      this.setColour(65);
   this.setTooltip("Get Data from IMU");
   this.setHelpUrl("");
    }
  };
Blockly.Blocks['IMU_TurnPID'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("TurnPID angle=");
    this.appendValueInput("S0")
      .setCheck("Number")
      .appendField("");
    this.appendValueInput("S1")
      .setCheck("Number")
      .appendField("Min speed=");
    this.appendValueInput("S2")
      .setCheck("Number")
      .appendField("Max speed=");
    this.appendValueInput("S3")
      .setCheck("Number")
      .appendField("KP=");
    this.appendValueInput("S4")
      .setCheck("Number")
      .appendField("KD=");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
    this.setTooltip("");
  }
};
Blockly.Blocks['IMU_TurnByDirection'] = {
  init: function() {
    this.appendDummyInput()
          .appendField("Turn to Direction=")
          .appendField(new Blockly.FieldDropdown([["⬆ turnNorth","0"],["➡ turnEast","1"], ["⬇ turnSouth","2"], ["⬅ turnWest","3"]]), "direction");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("Gyro update");
 this.setHelpUrl("");
  }
};
Blockly.Blocks['IMU_TurnByAngle'] = {
  init: function() {
    this.appendDummyInput()
          .appendField("Turn to angle =")
          .appendField(new Blockly.FieldDropdown([["90","0"],["-90","1"], ["180","2"], ["-180","3"]]), "angle");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
 this.setTooltip("");
 this.setHelpUrl("");
  }
};

Blockly.Blocks['IMU_moveStraightPID'] = {
  init: function() {
    
    this.appendDummyInput()
          .appendField("Move ")
          .appendField(new Blockly.FieldDropdown([["⬆ Forward","0"],["⬇ Backward","1"]]), "dir");
    this.appendValueInput("S0")
      .setCheck("Number")
      .appendField("Straight At Angle=");
    this.appendValueInput("S1")
      .setCheck("Number")
      .appendField("speed=");
    this.appendValueInput("S2")
      .setCheck("Number")
      .appendField("time=");
    this.appendValueInput("S3")
      .setCheck("Number")
      .appendField("KP=");
    this.appendValueInput("S4")
      .setCheck("Number")
      .appendField("KI=");
    this.appendValueInput("S5")
      .setCheck("Number")
      .appendField("KD=");
    this.appendDummyInput()
        .appendField("Slow before Stop")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "Slow");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
    this.setTooltip("");
  }
};
Blockly.Blocks['IMU_set_dataFor_turnDirection'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("Set Data for TurnDirection=");
    this.appendValueInput("S0")
      .setCheck("Number")
      .appendField("speed Min=");
    this.appendValueInput("S1")
      .setCheck("Number")
      .appendField("Max=");
    this.appendValueInput("S2")
      .setCheck("Number")
      .appendField("KP=");
    this.appendValueInput("S3")
      .setCheck("Number")
      .appendField("KD=");
    
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
    this.setTooltip("");
  }
};

Blockly.Blocks['IMU_moveStraightDirection'] = {
  init: function() {
    
    this.appendDummyInput()
          .appendField("Move ")
          .appendField(new Blockly.FieldDropdown([["⬆ Forward","0"],["⬇ Backward","1"]]), "dir");
    this.appendDummyInput()
          .appendField("At ")
          .appendField(new Blockly.FieldDropdown([["⬆ North","0"],["➡ East","1"], ["⬇ South","2"], ["⬅ West","3"]]), "angle");
    this.appendValueInput("S1")
      .setCheck("Number")
      .appendField("speed=");
    this.appendValueInput("S2")
      .setCheck("Number")
      .appendField("time=");
    this.appendValueInput("S3")
      .setCheck("Number")
      .appendField("KP=");
    this.appendValueInput("S4")
      .setCheck("Number")
      .appendField("KI=");
    this.appendValueInput("S5")
      .setCheck("Number")
      .appendField("KD=");
    this.appendDummyInput()
        .appendField("Slow before Stop")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "Slow");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
    this.setTooltip("");
  }
};




Blockly.Blocks['IMU_moveStraightPID_Encoder'] = {
  init: function() {
    
    this.appendDummyInput()
          .appendField("Move Encoder ")
          .appendField(new Blockly.FieldDropdown([["⬆ Forward","0"],["⬇ Backward","1"]]), "dir");
    this.appendValueInput("S0")
      .setCheck("Number")
      .appendField("Straight At Angle=");
    this.appendValueInput("S1")
      .setCheck("Number")
      .appendField("speed=");
    this.appendValueInput("S2")
      .setCheck("Number")
      .appendField("Pulse=");
    this.appendValueInput("S3")
      .setCheck("Number")
      .appendField("KP=");
    this.appendValueInput("S4")
      .setCheck("Number")
      .appendField("KI=");
    this.appendValueInput("S5")
      .setCheck("Number")
      .appendField("KD=");
    this.appendDummyInput()
        .appendField("Slow before Stop")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "Slow");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(67);
    this.setTooltip("");
  }
};
Blockly.Blocks['IMU_moveStraightDirection_Encoder'] = {
  init: function() {
    
    this.appendDummyInput()
          .appendField("Move Encoder ")
          .appendField(new Blockly.FieldDropdown([["⬆ Forward","0"],["⬇ Backward","1"]]), "dir");
    this.appendDummyInput()
          .appendField("At ")
          .appendField(new Blockly.FieldDropdown([["⬆ North","0"],["➡ East","1"], ["⬇ South","2"], ["⬅ West","3"]]), "angle");
    this.appendValueInput("S1")
      .setCheck("Number")
      .appendField("speed=");
    this.appendValueInput("S2")
      .setCheck("Number")
      .appendField("Pulse=");
    this.appendValueInput("S3")
      .setCheck("Number")
      .appendField("KP=");
    this.appendValueInput("S4")
      .setCheck("Number")
      .appendField("KI=");
    this.appendValueInput("S5")
      .setCheck("Number")
      .appendField("KD=");
    this.appendDummyInput()
        .appendField("Slow before Stop")
        .appendField(new Blockly.FieldCheckbox("TRUE"), "Slow");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(67);
    this.setTooltip("");
  }
};

Blockly.Blocks['reset_Encoder'] = {
  init: function() {
    this.appendDummyInput()
        .appendField("reset_Encoder to 0");
    this.setInputsInline(true);
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(65);
    this.setTooltip("");
  }
};
Blockly.Blocks['read_Encoder'] = {
    init: function() {
      this.appendDummyInput()
          .appendField("read_Encoder Pin D2 and D3");
      this.setInputsInline(true);
      this.setOutput(true, "Number");
      this.setColour(65);
   this.setTooltip("read_Encoder");
   this.setHelpUrl("");
    }
  };


}