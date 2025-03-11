const dirIcon = Vue.prototype.$global.board.board_info.dir;
module.exports = {
    base_blocks : [ // use "blocks : [ " in normally situation but this need to override base block from esp-idf platforms
    	        {
            name : 'PuppyBot',
            index: 1,
            color : '230',
            icon: `file:///${dirIcon}/static/cover.jpg`,
            blocks : [
                'sw1_press',
                'sw1_press_one',
                'button_1_status',
                'Light_Sensor',
                {
                    xml : 
                    `<block type="INPUT_digital">
                        <value name="pin">
                            <shadow type="math_number">
                                <field name="NUM">1</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml: `<sep gap="25"></sep><label text="ถ้าใช้งาน color sensor ตัวที่1 จะต้องต่อ SDA SCL ตามปกติ" web-class="main-content"></label>`
                },
                {
                    xml: `<sep gap="25"></sep><label text="ถ้าใช้งาน color sensor ตัวที่2 จะต้องต่อ SDA=Servo3 SCL=Servo4" web-class="main-content"></label>`
                },
                {
                    xml : 
                    `<block type="TCS_color_status">
                        <value name="_color">
                            <shadow type="math_number">
                                <field name="NUM">500</field>
                            </shadow>
                        </value>
                    </block>`
                },
                
                {
                    xml : 
                    `<block type="TCS_color_status_B">
                        <value name="_color">
                            <shadow type="math_number">
                                <field name="NUM">500</field>
                            </shadow>
                        </value>
                    </block>`
                },
                'Ultrasonic_sensor',
                {
                    xml: `<sep gap="25"></sep><label text="ใช้งาน IMU รุ่นใหม่ " web-class="main-content"></label>`
                },
                'IMU_begin',
                'IMU_getData',
                'IMU_getData_Yaw',
                'IMU_TurnByAngle',
                {xml:
                      `<block type="IMU_set_dataFor_turnDirection">
                            <value name="S0">
                                <shadow type="math_number">
                                    <field name="NUM">15</field>
                                </shadow>
                            </value>
                            <value name="S1">
                                <shadow type="math_number">
                                    <field name="NUM">60</field>
                                </shadow>
                            </value>
                            <value name="S2">
                                <shadow type="math_number">
                                    <field name="NUM">4</field>
                                </shadow>
                            </value>
                            <value name="S3">
                                <shadow type="math_number">
                                    <field name="NUM">5</field>
                                </shadow>
                            </value>
                        </block>`
              },
                'IMU_TurnByDirection',
                {xml:
                      `<block type="IMU_TurnPID">
                            <value name="S0">
                                <shadow type="math_number">
                                    <field name="NUM">90</field>
                                </shadow>
                            </value>
                            <value name="S1">
                                <shadow type="math_number">
                                    <field name="NUM">15</field>
                                </shadow>
                            </value>
                            <value name="S2">
                                <shadow type="math_number">
                                    <field name="NUM">60</field>
                                </shadow>
                            </value>
                            <value name="S3">
                                <shadow type="math_number">
                                    <field name="NUM">5</field>
                                </shadow>
                            </value>
                            <value name="S4">
                                <shadow type="math_number">
                                    <field name="NUM">5</field>
                                </shadow>
                            </value>
                        </block>`
              },
              {xml:
                      `<block type="IMU_moveStraightPID">
                            <value name="S0">
                                <shadow type="math_number">
                                    <field name="NUM">90</field>
                                </shadow>
                            </value>
                            <value name="S1">
                                <shadow type="math_number">
                                    <field name="NUM">50</field>
                                </shadow>
                            </value>
                            <value name="S2">
                                <shadow type="math_number">
                                    <field name="NUM">2000</field>
                                </shadow>
                            </value>
                            <value name="S3">
                                <shadow type="math_number">
                                    <field name="NUM">1</field>
                                </shadow>
                            </value>
                            <value name="S4">
                                <shadow type="math_number">
                                    <field name="NUM">0</field>
                                </shadow>
                            </value>
                             <value name="S5">
                                <shadow type="math_number">
                                    <field name="NUM">0.5</field>
                                </shadow>
                            </value>
                        </block>`
              },
              {xml:
                      `<block type="IMU_moveStraightDirection">
                            <value name="S1">
                                <shadow type="math_number">
                                    <field name="NUM">50</field>
                                </shadow>
                            </value>
                            <value name="S2">
                                <shadow type="math_number">
                                    <field name="NUM">2000</field>
                                </shadow>
                            </value>
                            <value name="S3">
                                <shadow type="math_number">
                                    <field name="NUM">1</field>
                                </shadow>
                            </value>
                            <value name="S4">
                                <shadow type="math_number">
                                    <field name="NUM">0</field>
                                </shadow>
                            </value>
                             <value name="S5">
                                <shadow type="math_number">
                                    <field name="NUM">0.5</field>
                                </shadow>
                            </value>
                        </block>`
              },
              {xml:
                      `<block type="IMU_moveStraightPID_Encoder">
                            <value name="S0">
                                <shadow type="math_number">
                                    <field name="NUM">90</field>
                                </shadow>
                            </value>
                            <value name="S1">
                                <shadow type="math_number">
                                    <field name="NUM">50</field>
                                </shadow>
                            </value>
                            <value name="S2">
                                <shadow type="math_number">
                                    <field name="NUM">1000</field>
                                </shadow>
                            </value>
                            <value name="S3">
                                <shadow type="math_number">
                                    <field name="NUM">1</field>
                                </shadow>
                            </value>
                            <value name="S4">
                                <shadow type="math_number">
                                    <field name="NUM">0</field>
                                </shadow>
                            </value>
                             <value name="S5">
                                <shadow type="math_number">
                                    <field name="NUM">0.5</field>
                                </shadow>
                            </value>
                        </block>`
              },
              {xml:
                      `<block type="IMU_moveStraightDirection_Encoder">
                            <value name="S1">
                                <shadow type="math_number">
                                    <field name="NUM">50</field>
                                </shadow>
                            </value>
                            <value name="S2">
                                <shadow type="math_number">
                                    <field name="NUM">1000</field>
                                </shadow>
                            </value>
                            <value name="S3">
                                <shadow type="math_number">
                                    <field name="NUM">1</field>
                                </shadow>
                            </value>
                            <value name="S4">
                                <shadow type="math_number">
                                    <field name="NUM">0</field>
                                </shadow>
                            </value>
                             <value name="S5">
                                <shadow type="math_number">
                                    <field name="NUM">0.5</field>
                                </shadow>
                            </value>
                        </block>`
              },
              'read_Encoder',
              'reset_Encoder',
                
            ]
        },
        //display
        {
            name: "Display",
            index: 2,
            color: "230",
            icon: "/static/icons/icons8_picture_96px_1.png",
            blocks: [
                "tft_display_setRotation",
                "tft_display_fillScreen",
                {
                    xml:
                        `<block type="tft_display_print">
                                    <value name="text">
                                        <shadow type="basic_string">
                                            <field name="VALUE">Hello world!</field>
                                        </shadow>
                                    </value>
                                    <value name="x">
                                        <shadow type="math_number">
                                            <field name="NUM">0</field>
                                        </shadow>
                                    </value>
                                    <value name="y">
                                        <shadow type="math_number">
                                            <field name="NUM">0</field>
                                        </shadow>
                                    </value>
                                    <value name="size">
                                        <shadow type="math_number">
                                            <field name="NUM">2</field>
                                        </shadow>
                                    </value>
                                </block>`
                },
                {
                    xml:
                        `<block type="tft_display_print2">
                                    <value name="text">
                                        <shadow type="basic_string">
                                            <field name="VALUE">Hello world!</field>
                                        </shadow>
                                    </value>
                                    <value name="x">
                                        <shadow type="math_number">
                                            <field name="NUM">0</field>
                                        </shadow>
                                    </value>
                                    <value name="y">
                                        <shadow type="math_number">
                                            <field name="NUM">0</field>
                                        </shadow>
                                    </value>
                                    <value name="size">
                                        <shadow type="math_number">
                                            <field name="NUM">2</field>
                                        </shadow>
                                    </value>
                                </block>`
                },
                {
                    xml:
                        `<block type="tft_display_draw_line">
                                    <value name="x0">
                                        <shadow type="math_number">
                                            <field name="NUM">10</field>
                                        </shadow>
                                    </value>
                                    <value name="y0">
                                        <shadow type="math_number">
                                            <field name="NUM">10</field>
                                        </shadow>
                                    </value>
                                    <value name="x1">
                                        <shadow type="math_number">
                                            <field name="NUM">100</field>
                                        </shadow>
                                    </value>
                                    <value name="y1">
                                        <shadow type="math_number">
                                            <field name="NUM">50</field>
                                        </shadow>
                                    </value>
                                </block>`
                },
                {
                    xml:
                        `<block type="tft_display_draw_rect">
                                    <value name="x">
                                        <shadow type="math_number">
                                            <field name="NUM">10</field>
                                        </shadow>
                                    </value>
                                    <value name="y">
                                        <shadow type="math_number">
                                            <field name="NUM">10</field>
                                        </shadow>
                                    </value>
                                    <value name="width">
                                        <shadow type="math_number">
                                            <field name="NUM">50</field>
                                        </shadow>
                                    </value>
                                    <value name="height">
                                        <shadow type="math_number">
                                            <field name="NUM">30</field>
                                        </shadow>
                                    </value>
                                </block>`
                },
                {
                    xml:
                        `<block type="tft_display_draw_circle">
                                    <value name="x">
                                        <shadow type="math_number">
                                            <field name="NUM">64</field>
                                        </shadow>
                                    </value>
                                    <value name="y">
                                        <shadow type="math_number">
                                            <field name="NUM">32</field>
                                        </shadow>
                                    </value>
                                    <value name="r">
                                        <shadow type="math_number">
                                            <field name="NUM">20</field>
                                        </shadow>
                                    </value>
                                </block>`
                },
                "basic_string"
            ]
        },
        //motor
        {
            name : 'Motor',
            index: 3,
            color : '230',
            //icon: "/static/icons/icons8_workflow_128px.png",
            icon: `file:///${dirIcon}/static/electric-motor.png`,
            blocks : [

                
                { 
                    xml : 
                    `<block type="PuppyBot_servo">
                        <value name="ch">
                            <shadow type="math_number">
                                <field name="NUM">1</field>
                            </shadow>
                        </value>
                        <value name="angle">
                            <shadow type="math_number">
                                <field name="NUM">90</field>
                            </shadow>
                        </value>
                    </block>`
                },
                { 
                    xml : 
                    `<block type="PuppyBot_servo_speed_control">
                        <value name="ch">
                            <shadow type="math_number">
                                <field name="NUM">1</field>
                            </shadow>
                        </value>
                        <value name="servo_degree">
                            <shadow type="math_number">
                                <field name="NUM">90</field>
                            </shadow>
                        </value>
                        <value name="traget_degree">
                            <shadow type="math_number">
                                <field name="NUM">120</field>
                            </shadow>
                        </value>
                        <value name="servo_speed">
                            <shadow type="math_number">
                                <field name="NUM">5</field>
                            </shadow>
                        </value>
                    </block>`
                },
                'PuppyBot_stop',
                {
                    xml : 
                    `<block type="PuppyBot_stop_time">
                        <value name="time">
                            <shadow type="math_number">
                                <field name="NUM">500</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="PuppyBot_stop_ch">
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="PuppyBotmotor2WD">
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml: `<sep gap="25"></sep><label text="ระบบขับเคลื่อน 4 ล้อ แบ่งเป็น M1=หน้าซ้าย M2=หน้าขวา M3=หลังซ้าย M4=หลังขวา" web-class="main-content"></label>`
                },
                {
                    xml : 
                    `<block type="PuppyBotmotor4WD">
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="PuppyBotmotor_Mecanum">
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="PuppyBot_forward">
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="PuppyBot_backward">
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="PuppyBot_spin_left">
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="PuppyBot_spin_right">
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="PuppyBot_turn_left">
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="PuppyBot_turn_right">
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                { 
                    xml : 
                    `<block type="PuppyBot">
                        <value name="ch">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                        <value name="dir">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                        <value name="speed">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                { 
                    xml : 
                    `<block type="PuppyBot_forward2">
                        <value name="speed1">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                        <value name="speed2">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                { 
                    xml : 
                    `<block type="PuppyBot_backward2">
                        <value name="speed1">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                        <value name="speed2">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                { 
                    xml : 
                    `<block type="PuppyBot_forward2_4WD">
                        <value name="speed1">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                        <value name="speed2">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                { 
                    xml : 
                    `<block type="PuppyBot_backward2_4WD">
                        <value name="speed1">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                        <value name="speed2">
                            <shadow type="math_number">
                                <field name="NUM">50</field>
                            </shadow>
                        </value>
                    </block>`
                },
                

        
                

                ]
        },
        //PID
        {
            name : 'PID',
            index: 4,
            color : '230',
            //icon: "/static/icons/icons8_workflow_128px.png",
            icon: `file:///${dirIcon}/static/PID.png`,
            blocks : [
               
              {
                xml: `<sep gap="25"></sep><label text="1.ตั้งค่าการเชื่อมต่อเซ็นเซอร์ เริ่มจากซ้ายไปขวา" web-class="headline"></label>`
              },
              {
                xml: `<sep gap="25"></sep><label text="2.ตั้งค่า Min = ค่าที่อ่านจากพื้นสีดำ" web-class="headline"></label>`
              },
              {
                xml: `<sep gap="25"></sep><label text="3.ตั้งค่า Max = ค่าที่อ่านจากพื้นสีขาว" web-class="headline"></label>`
              },
                {xml:`<block type="PuppyBot_PID_setPin"></block>`},
                'PuppyBot_PID_setline_color_front',
              //   {xml:
              //         `<block type="set_Sensitive_Front_sensor">
              //               <value name="Sensitive">
              //                   <shadow type="math_number">
              //                       <field name="NUM">80</field>
              //                   </shadow>
              //               </value>
                            
              //           </block>`
              // },
	          
	          {xml:
	                  `<block type="PuppyBot_PID_setMin">
	                        <value name="S0">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S1">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S2">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S3">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S4">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S5">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S6">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S7">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        
	                    </block>`
	          },
	          {
	              xml:
	                  `<block type="PuppyBot_PID_setMax">
	                        <value name="S0">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S1">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S2">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S3">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S4">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S5">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S6">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S7">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        
	                    </block>`
	          },
	          {xml:
                      `<block type="set_calibrate_Front_sensor">
                            <value name="Round">
                                <shadow type="math_number">
                                    <field name="NUM">2000</field>
                                </shadow>
                            </value>
                            
                        </block>`
              },

	          {
	              xml:
	                  `<block type="PuppyBot_Run_PID">
	                        <value name="speed">
	                            <shadow type="math_number">
	                                <field name="NUM">30</field>
	                            </shadow>
	                        </value>
	                        <value name="KP">
	                            <shadow type="math_number">
	                                <field name="NUM">0.2</field>
	                            </shadow>
	                        </value>
	                        <value name="KD">
	                            <shadow type="math_number">
	                                <field name="NUM">0</field>
	                            </shadow>
	                        </value>
	                    </block>`
	          },
              {
                  xml:
                      `<block type="PuppyBot_Run_PID_until_frontSensor">
                            <value name="speed">
                                <shadow type="math_number">
                                    <field name="NUM">30</field>
                                </shadow>
                            </value>
                            <value name="KP">
                                <shadow type="math_number">
                                    <field name="NUM">0.2</field>
                                </shadow>
                            </value>
                            <value name="KD">
                                <shadow type="math_number">
                                    <field name="NUM">0</field>
                                </shadow>
                            </value>
                            <value name="sumSensor">
                                <shadow type="math_number">
                                    <field name="NUM">200</field>
                                </shadow>
                            </value>
                        </block>`
              },
              // {xml:`<block type="Read_Min_Front_Sensor"></block>`},
               
              
              'Read_Status_Front_Sensor',
              "Read_Ref_Front_Sensor",
              
              'PID_readSumSensor',
              "PID_readLine",

	          {xml:`<block type="PuppyBot_PID_setPin_B"></block>`},
              'PuppyBot_PID_setline_color_Black',
	          
	          {xml:
	                  `<block type="PuppyBot_PID_setMin_B">
	                        <value name="S0">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S1">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S2">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S3">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S4">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S5">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S6">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        <value name="S7">
	                            <shadow type="math_number">
	                                <field name="NUM">100</field>
	                            </shadow>
	                        </value>
	                        
	                    </block>`
	          },
	          {
	              xml:
	                  `<block type="PuppyBot_PID_setMax_B">
	                        <value name="S0">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S1">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S2">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S3">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S4">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S5">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S6">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        <value name="S7">
	                            <shadow type="math_number">
	                                <field name="NUM">1023</field>
	                            </shadow>
	                        </value>
	                        
	                    </block>`
	          },
	          {xml:
                      `<block type="set_calibrate_Back_sensor">
                            <value name="Round">
                                <shadow type="math_number">
                                    <field name="NUM">2000</field>
                                </shadow>
                            </value>
                            
                        </block>`
              },
              {
	              xml:
	                  `<block type="PuppyBot_Run_PID_B">
	                        <value name="speed">
	                            <shadow type="math_number">
	                                <field name="NUM">30</field>
	                            </shadow>
	                        </value>
	                        <value name="KP">
	                            <shadow type="math_number">
	                                <field name="NUM">0.2</field>
	                            </shadow>
	                        </value>
	                        <value name="KD">
	                            <shadow type="math_number">
	                                <field name="NUM">0</field>
	                            </shadow>
	                        </value>
	                    </block>`
	          },
              {
                  xml:
                      `<block type="PuppyBot_Run_PID_B_until_backSensor">
                            <value name="speed">
                                <shadow type="math_number">
                                    <field name="NUM">30</field>
                                </shadow>
                            </value>
                            <value name="KP">
                                <shadow type="math_number">
                                    <field name="NUM">0.2</field>
                                </shadow>
                            </value>
                            <value name="KD">
                                <shadow type="math_number">
                                    <field name="NUM">0</field>
                                </shadow>
                            </value>
                            <value name="sumSensor">
                                <shadow type="math_number">
                                    <field name="NUM">200</field>
                                </shadow>
                            </value>
                        </block>`
              },
	          // {xml:
           //            `<block type="set_Sensitive_Back_sensor">
           //                  <value name="Sensitive">
           //                      <shadow type="math_number">
           //                          <field name="NUM">80</field>
           //                      </shadow>
           //                  </value>
                            
           //              </block>`
           //    },
              
              'Read_Status_Back_Sensor',
              "Read_Ref_Back_Sensor",
              'PID_readSumSensor_B',
              "PID_readLine_B",

                ]
        },
        // //GPIO
        // {
        //     name : 'GPIO',
        //     index: 5,
        //     color : '230',
        //     icon : '/static/icons/icons8_electronics_96px.png',
        //     blocks : [
        //         {
        //             xml : 
        //             `<block type="io_setpin">
        //                 <value name="pin">
        //                     <shadow type="math_number">
        //                         <field name="NUM">25</field>
        //                     </shadow>
        //                 </value>
        //             </block>`
        //         },
        //         {
        //             xml : 
        //             `<block type="io_digital_read">
        //                 <value name="pin">
        //                     <shadow type="math_number">
        //                         <field name="NUM">32</field>
        //                     </shadow>
        //                 </value>
        //             </block>`
        //         },
        //         {
        //             xml : 
        //             `<block type="io_digital_write">
        //                 <value name="pin">
        //                     <shadow type="math_number">
        //                         <field name="NUM">33</field>
        //                     </shadow>
        //                 </value>
        //                 <value name="value">
        //                     <shadow type="math_number">
        //                         <field name="NUM">1</field>
        //                     </shadow>
        //                 </value>
        //             </block>`
        //         },                
        //         {
        //             xml : 
        //             `<block type="io_analog_read">
        //                 <value name="pin">
        //                     <shadow type="math_number">
        //                         <field name="NUM">36</field>
        //                     </shadow>
        //                 </value>
        //             </block>`
        //         },
        //         {
        //             xml : 
        //             `<block type="io_pwm_write">
        //                 <value name="pin">
        //                     <shadow type="math_number">
        //                         <field name="NUM">36</field>
        //                     </shadow>
        //                 </value>
        //                 <value name="value">
        //                     <shadow type="math_number">
        //                         <field name="NUM">128</field>
        //                     </shadow>
        //                 </value>
        //             </block>`
        //         },
        //         {
        //             xml : 
        //             `<block type="io_pulse_in">
        //                 <value name="pin">
        //                     <shadow type="math_number">
        //                         <field name="NUM">36</field>
        //                     </shadow>
        //                 </value>
        //             </block>`
        //         },
        //         'io_shift_in',
        //         {
        //             xml : 
        //             `<block type="io_shift_out">
        //                 <value name="data">
        //                     <shadow type="math_number">
        //                         <field name="NUM">127</field>
        //                     </shadow>
        //                 </value>
        //             </block>`
        //         }
        //     ]
        // },
        //Music  
        {
            name: "Music",
            index: 7,
            color: "330",
            icon: "/static/icons/SVG/c6.svg",
            blocks: [
              "music_note",
              "music_notes",
              {
                xml:
                  `<block type="music_play_notes">
                              <value name="note">                    
                                  <block type="music_notes">
                                      <field name="notes">C4,B4,E4</field>
                                  </block>
                              </value>
                          </block>`
              },
              'music_song_mario_underworld',
              'music_song_jingle_bell',
              'music_song_cannon_rock'
              // 'music_rest',
              // 'music_scale',
              // 'music_set_volume',
              // 'music_get_volume'
            ]
          },      
        //Time
        {
            name : 'Time',
            color : '230',
            index: 6,
            icon : '/static/icons/icons8_Story_Time_96px.png',
            blocks : [
                {
                    xml : 
                    `<block type="time_delay">
                        <value name="delay">
                            <shadow type="math_number">
                                <field name="NUM">500</field>
                            </shadow>
                        </value>
                    </block>`
                },
                'time_millis',
                'time_micros'
            ]
        },        
        //Variables
        {
            name : 'Variables',
            index: 8,
            color : '230',
            icon : '/static/icons/icons8_variable_96px.png',
            custom : 'VARIABLE'
        },
        //Math
        {
            name : 'Math',
            index: 9,
            color : '230',
            icon : '/static/icons/calculator.png',
            blocks : [
                'math_number',
                {
                    xml : 
                    `<block type="math_arithmetic">
                        <value name="A">
                            <shadow type="math_number">
                                <field name="NUM">1</field>
                            </shadow>
                        </value>
                        <value name="B">
                            <shadow type="math_number">
                                <field name="NUM">1</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="math_variables_set">
                        <value name="VALUE">
                            <shadow type="math_number">
                                <field name="NUM">1</field>
                            </shadow>
                        </value>
                    </block>`
                },
                'math_variables_get',
                {
                    xml : 
                    `<block type="math_pow">
                        <value name="NUM1">
                            <shadow type="math_number">
                                <field name="NUM">2</field>
                            </shadow>
                        </value>
                        <value name="NUM2">
                            <shadow type="math_number">
                                <field name="NUM">3</field>
                            </shadow>
                        </value>
                    </block>`
                },                
                /*'math_sqrt',*/
                {
                    xml : 
                    `<block type="math_single">
                        <value name="NUM">
                            <shadow type="math_number">
                                <field name="NUM">9</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="math_trig">
                        <value name="NUM">
                            <shadow type="math_number">
                                <field name="NUM">90</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="math_round">
                        <value name="NUM">
                            <shadow type="math_number">
                                <field name="NUM">1.2</field>
                            </shadow>
                        </value>
                    </block>`
                },
                /*'math_min',
                'math_max',
                'math_map',*/
                'math_random_int',
                {
                    xml : 
                    `<block type="math_number_property">
                        <value name="NUMBER_TO_CHECK">
                            <shadow type="math_number">
                                <field name="NUM">5</field>
                            </shadow>
                        </value>
                    </block>`
                }
            ]
        },
        //Logic
        {
            name : 'Logic',
            index: 10,
            color : '230',
            icon : '/static/icons/icons8_serial_tasks_96px.png',
            blocks : [
                'controls_if',
                {
                    xml : '<block type="controls_if"><mutation else="1"></mutation></block>'
                },
                'logic_compare',
                'logic_operation',
                'logic_negate',
                'logic_boolean',
            ]
        },
        //Loops
        {
            name : 'Loops',
            index: 11,
            color : '230',
            icon : '/static/icons/icons8_repeat_96px.png',
            blocks : [
                'basic_forever',
                'controls_whileUntil',
                
                {
                    xml : 
                    `<block type="basic_forever_timeout">
                        <value name="timer">
                            <shadow type="math_number">
                                <field name="NUM">100</field>
                            </shadow>
                        </value>
                    </block>`
                },
                {
                    xml : 
                    `<block type="controls_for">
                        <value name="FROM">
                            <shadow type="math_number">
                                <field name="NUM">1</field>
                            </shadow>
                        </value>
                        <value name="TO">
                            <shadow type="math_number">
                                <field name="NUM">10</field>
                            </shadow>
                        </value>
                        <value name="BY">
                            <shadow type="math_number">
                                <field name="NUM">1</field>
                            </shadow>
                        </value>
                    </block>`
                },
                'controls_flow_statements',
            ]
        },
        //Advanced
        {
            name : 'Advanced',
            index: 12,
            color : '195',
            icon : '/static/icons/icons8_hacker_128px.png',
            blocks : [
                {
                    type : 'category',
                    name : 'Functions',
                    icon : '/static/icons/icons8_module_96px.png',
                    custom : 'PROCEDURE'
                },
                /*{
                    type : 'category',
                    name : 'Tasks',
                    icon : '/static/icons/icons8_exercise_96px.png',
                    blocks : [
                        'create task',
                        'start task',
                        'stop task'
                    ]
                },*/
                /*{
                    type : 'category',
                    name : 'Arrays',
                    icon : '/static/icons/icons8_stack_96px.png',
                    blocks : [
                        'lists_create_empty',
                        'lists_repeat',
                        'lists_reverse',
                        'lists_isEmpty',
                        'lists_length',
                        'lists_create_with',
                        'lists_indexOf',
                        'lists_getIndex',
                        'lists_setIndex',
                        'lists_getSublist',
                        'lists_sort',
                        'lists_split',
                    ]
                },*/
                {
                    type : 'category',
                    name : 'Text',
                    icon : '/static/icons/icons8_text_color_96px.png',
                    blocks : [
                        'basic_string',                        
                        { xml : 
                            `<block type="text_length">
                                <value name="VALUE">
                                    <shadow type="basic_string">
                                        <field name="VALUE">Hello world!</field>
                                    </shadow>
                                </value>                                    
                            </block>`
                        },
                        'text_join',                        
                        { xml : 
                            `<block type="text_append">
                                <value name="TEXT">
                                    <shadow type="basic_string">
                                        <field name="VALUE">Hello world!</field>
                                    </shadow>
                                </value>                                    
                            </block>`
                        },
                        { xml : 
                                `<block type="text_isEmpty">
                                    <value name="VALUE">
                                        <shadow type="basic_string">
                                            <field name="VALUE">Hello world!</field>
                                        </shadow>
                                    </value>                                    
                                </block>`
                        },
                        { xml : 
                            `<block type="text_indexOf">
                                <value name="VALUE">
                                    <shadow type="basic_string">
                                        <field name="VALUE">Hello world!</field>
                                    </shadow>
                                </value>                                    
                            </block>`
                        },
                        { xml : 
                            `<block type="text_charAt">
                                <value name="VALUE">
                                    <shadow type="basic_string">
                                        <field name="VALUE">Hello world!</field>
                                    </shadow>
                                </value>                                    
                            </block>`
                        },
                        { xml : 
                            `<block type="text_getSubstring">
                                <value name="STRING">
                                    <shadow type="basic_string">
                                        <field name="VALUE">Hello world!</field>
                                    </shadow>
                                </value>                                    
                            </block>`
                        },
                        { xml : 
                            `<block type="text_changeCase">
                                <value name="TEXT">
                                    <shadow type="basic_string">
                                        <field name="VALUE">Hello world!</field>
                                    </shadow>
                                </value>                                    
                            </block>`
                        },
                        { xml : 
                            `<block type="text_trim">
                                <value name="TEXT">
                                    <shadow type="basic_string">
                                        <field name="VALUE">Hello world!</field>
                                    </shadow>
                                </value>                                    
                            </block>`
                        },
                        { xml : 
                            `<block type="text_replace">
                                <value name="TEXT">
                                    <shadow type="basic_string">
                                        <field name="VALUE">Hello world!</field>
                                    </shadow>
                                </value>                                    
                            </block>`
                        },
                        //'text_compare',
                        //'text_parse_int'
                    ]
                },
                {
                    type : 'category',
                    name : 'Serial',
                    icon : '/static/icons/SVG/13.svg',
                    blocks : [
                        'serial_usb_init',
                        'serial_hardware_init',
                        'serial_available',
                        {
                            xml : 
                                `<block type="serial_write_data">
                                    <value name="text">
                                        <shadow type="basic_string">
                                            <field name="VALUE">Hello world!</field>
                                        </shadow>
                                    </value>
                                </block>`
                        },
                        'serial_write_newline',                        
                        'serial_read_line',
                        'serial_read_until',
                        'basic_string'
                    ]
                }
            ]
        }
    ]
}