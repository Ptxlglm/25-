//多轴电机驱动器控制程序，6路步进电机2路直流电机，串口命令控制
//2022.12.27

// 全局设置部分
#define UP 1                       //定义电机方向（正转）
#define DOWN -1                    //定义电机方向（反转）
#define FREE 0                     //电机自由停止状态
#define STOP -2                    //电机刹车状态
int led2 = 31;
int led3 = 32;
int led4 = 33;
#define CMD_LENGTH 14                  //定义命令缓冲区长度
char Cmd_string[CMD_LENGTH]={0,0};    //设置变量存储当前命令

// 步进电机配置
//步进电机管脚定义：步进电机控制引脚定义（脉冲、方向、使能）
#define TOTAL_MOTOR_NUM 6 
//定义步进电机控制端口号地址
char MOTOR_PUL_ARRAY[TOTAL_MOTOR_NUM]={39,40,38,17,16,15};
char MOTOR_DIR_ARRAY[TOTAL_MOTOR_NUM]={27,25,30,47,9,87};
char MOTOR_ENA_ARRAY[TOTAL_MOTOR_NUM]={65,74,66,41,86,85};
// char MOTOR_PUL_ARRAY[TOTAL_MOTOR_NUM]={24,30,36,42,48,25};
// char MOTOR_DIR_ARRAY[TOTAL_MOTOR_NUM]={26,32,38,44,50,27};
// char MOTOR_ENA_ARRAY[TOTAL_MOTOR_NUM]={22,28,34,40,46,23};
//char MOTOR_PUL_ARRAY[TOTAL_MOTOR_NUM]={45,44,46,17,16,15};
//char MOTOR_DIR_ARRAY[TOTAL_MOTOR_NUM]={27,12,30,47,9,87};
//char MOTOR_ENA_ARRAY[TOTAL_MOTOR_NUM]={65,26,66,41,86,85};

// 步数计数器
int STEP_NUM_COUNT_ARRAY[TOTAL_MOTOR_NUM]={0,0};

// 直流电机配置
//直流电机管脚定义：直流电机控制引脚（PWM、方向1、方向2）
#define TOTAL_DRILL_MOTOR_NUM 2  //定义直流电机数量
#define DRILL_MOTOR_STBY 52  //使能引脚？？？
//定义直流电机控制端口号地址
char DRILL_MOTOR_PWM_ARRAY[TOTAL_DRILL_MOTOR_NUM]={2,3};
char DRILL_MOTOR_IN1_ARRAY[TOTAL_DRILL_MOTOR_NUM]={29,33};
char DRILL_MOTOR_IN2_ARRAY[TOTAL_DRILL_MOTOR_NUM]={31,35};


//创建变量存储控制电机序号，用于多轴联动控制
char MOTOR_num_array[6]={0,1,2,3,4,5};    //电机序列数组
char MOTOR_dir_array[6]={0,1,2,3,4,5};       //命令方向数组
int MOTOR_step_array[6]={0,1,2,3,4,5};       //电机步数数组
int MOTOR_speedtime_array[6]={0,1,2,3,4,5};  //电机速度数组
char CMD_Count_flag = 0;                    //记录收到命令条数，用于多轴联动
/************************************************************/
//设置电机方向
void Set_Motor_Dri(int Dri, char MOTOR_Num)
{
  if (Dri==DOWN)
  {
    digitalWrite(MOTOR_DIR_ARRAY[MOTOR_Num], HIGH);    //方向
    }

  if (Dri==UP)
  {
    digitalWrite(MOTOR_DIR_ARRAY[MOTOR_Num], LOW);     //方向
    }
}
/************************************************************/
/*电机运动函数*/ // 单个电机控制
void Motor_Move_count( int step_num, int Dri, char MOTOR_Num, int Speed_Time)
{
  Set_Motor_Dri(Dri, MOTOR_Num);  //设置电机方向
  digitalWrite(MOTOR_ENA_ARRAY[MOTOR_Num], LOW);//使能电机
  
  // 发送脉冲序列
  for(int i=0;i<step_num;i++)
  {
      digitalWrite(MOTOR_PUL_ARRAY[MOTOR_Num], HIGH);       // turn the LED on (HIGH is the voltage level)
      delayMicroseconds(Speed_Time);  // 延时，有控制脉冲宽度（速度）的作用// wait for a second,微妙级延时
      //delay(speed_time);                                    // wait for a second,毫妙级延时
      digitalWrite(MOTOR_PUL_ARRAY[MOTOR_Num], LOW);        //turn the LED on (HIGH is the voltage level)
      delayMicroseconds(Speed_Time);                        // wait for a second,微妙级延时
      //delay(Speed_Time);                                 // wait for a second,毫妙级延时

      // 更新步数计数器
      if (Dri==UP)    //正向计步
      {
        STEP_NUM_COUNT_ARRAY[MOTOR_Num]++;
      }
      if (Dri==DOWN)      //反向计步
      {
        STEP_NUM_COUNT_ARRAY[MOTOR_Num]--;
      }
  }
  //Serial.println(STEP_NUM_COUNT_ARRAY[MOTOR_Num]); 
  digitalWrite(MOTOR_ENA_ARRAY[MOTOR_Num], HIGH);  //禁能电机：禁用电机
}

/************************************************************/
/*机器人直线运动函数*/
void Robot_Move_XY_count( int step_num, int Dri, char XY_Num, int Speed_Time)
{
  char MOTOR_Num_0;
  char MOTOR_Num_1;
  
  if (XY_Num=='X')
  {
    MOTOR_Num_0=0;
    MOTOR_Num_1=2;
  }
  else if(XY_Num =='Y')
  {
  MOTOR_Num_0=1;
  MOTOR_Num_1=3;
  }

  Set_Motor_Dri(Dri, MOTOR_Num_0);                            //设置电机1方向
  Set_Motor_Dri(Dri, MOTOR_Num_1);                            //设置电机2方向
  
  digitalWrite(MOTOR_ENA_ARRAY[MOTOR_Num_0], LOW);//电机使能
  digitalWrite(MOTOR_ENA_ARRAY[MOTOR_Num_1], LOW);//电机使能
  
  for(int i=0;i<step_num;i++)
  {
      digitalWrite(MOTOR_PUL_ARRAY[MOTOR_Num_0], HIGH);       // turn the LED on (HIGH is the voltage level)
      digitalWrite(MOTOR_PUL_ARRAY[MOTOR_Num_1], HIGH);       // turn the LED on (HIGH is the voltage level)
      
      delayMicroseconds(Speed_Time);                      // wait for a second,微妙级延时
      //delay(speed_time);                                    // wait for a second,毫妙级延时
      digitalWrite(MOTOR_PUL_ARRAY[MOTOR_Num_0], LOW);        //turn the LED on (HIGH is the voltage level)
      digitalWrite(MOTOR_PUL_ARRAY[MOTOR_Num_1], LOW);        //turn the LED on (HIGH is the voltage level)
      delayMicroseconds(Speed_Time);                        // wait for a second,微妙级延时
      //delay(Speed_Time);                                 // wait for a second,毫妙级延时

      if (Dri==UP)    //正向计步
      {
        STEP_NUM_COUNT_ARRAY[MOTOR_Num_0]++;
        STEP_NUM_COUNT_ARRAY[MOTOR_Num_1]++;
      }
      if (Dri==DOWN)      //反向计步
      {
        STEP_NUM_COUNT_ARRAY[MOTOR_Num_0]--;
        STEP_NUM_COUNT_ARRAY[MOTOR_Num_1]--;
      }
  }
  //Serial.println(STEP_NUM_COUNT_ARRAY[MOTOR_Num_0]); 
  //Serial.println(STEP_NUM_COUNT_ARRAY[MOTOR_Num_1]); 
  
  digitalWrite(MOTOR_ENA_ARRAY[MOTOR_Num_0], HIGH);//电机禁能
  digitalWrite(MOTOR_ENA_ARRAY[MOTOR_Num_1], HIGH);//电机禁能
}
/************************************************************/
/*机器人多轴联动函数-测试调试用*/  // 多轴联动控制（Bresenham算法）
void Robot_Move_Multi_axis_count_test()
{
  int step_even_count; // 步数计数
  int step_0,step_1,step_2,step_3; //各轴输出的步数 ΔY
  int count_0,count_1,count_2,count_3;//各轴的Pi
  int step_completed=0;//步数计数器


  Set_Motor_Dri(DOWN, 0);                            //设置电机1方向
  Set_Motor_Dri(DOWN, 1);                            //设置电机2方向
  Set_Motor_Dri(DOWN, 2);                            //设置电机2方向
  Set_Motor_Dri(DOWN, 3);                            //设置电机2方向
  
  digitalWrite(MOTOR_ENA_ARRAY[0], LOW);//电机使能
  digitalWrite(MOTOR_ENA_ARRAY[1], LOW);//电机使能
  digitalWrite(MOTOR_ENA_ARRAY[2], LOW);//电机使能
  digitalWrite(MOTOR_ENA_ARRAY[3], LOW);//电机使能

  step_0=4000;
  step_1=8000;
  step_2=4000;
  step_3=8000;
  step_even_count = max(step_0,step_1);
  step_even_count = max(step_2,step_even_count);
  step_even_count = max(step_3,step_even_count);  //取各轴最大值作为X（ΔX）
  count_0 = -(step_even_count>>1);                //-ΔX/2  ΔX➗2，后边所有数据都是除2处理 
  count_1 = count_0;
  count_2 = count_0;
  count_3 = count_0;
 
 
  // ΔX都是一致的
  // < 0 Pi + ΔY
  // > 0 Pi + ΔY - ΔX
  //并且应该注意到 P1 = ΔY - ΔX/2
  
  while(step_completed<step_even_count)
  {
      count_0+=step_0;  //计算小于0 的情况 = Pi + ΔY
      if(count_0>0)    //Pi >0
      {
        digitalWrite(MOTOR_PUL_ARRAY[0], HIGH);
        count_0-=step_even_count; //由于Pi + ΔY已经计算，此处直接减去ΔX即可
        digitalWrite(MOTOR_PUL_ARRAY[0], LOW);
        }
 
       count_1+=step_1;
      if(count_1>0)
      {
        digitalWrite(MOTOR_PUL_ARRAY[1], HIGH);
        count_1-=step_even_count;
         digitalWrite(MOTOR_PUL_ARRAY[1], LOW);
        }
 
       count_2+=step_2;
      if(count_2>0)
      {
        digitalWrite(MOTOR_PUL_ARRAY[2], HIGH);
        count_2-=step_even_count;
        digitalWrite(MOTOR_PUL_ARRAY[2], LOW);
        }

      count_3+=step_3;
      if(count_3>0)
      {
        digitalWrite(MOTOR_PUL_ARRAY[3], HIGH);
        count_3-=step_even_count;
        digitalWrite(MOTOR_PUL_ARRAY[3], LOW);
       }
        
   step_completed+=1;
   delayMicroseconds(150);//脉冲间隔
    }

  digitalWrite(MOTOR_ENA_ARRAY[0], HIGH);//电机禁能
  digitalWrite(MOTOR_ENA_ARRAY[1], HIGH);//电机禁能
  digitalWrite(MOTOR_ENA_ARRAY[2], HIGH);//电机禁能
  digitalWrite(MOTOR_ENA_ARRAY[3], HIGH);//电机禁能
}

/************************************************************/
/*机器人多轴联动函数-真实使用*/

void Robot_Move_Multi_axis_count(char MOTOR_dir_array[6],int MOTOR_step_array[6],int MOTOR_speedtime_array[6])
{
  int step_even_count=0; // 步数计数
  int deta_step[6]={0,0,0,0,0,0}; //各轴输出的步数 ΔY
  int step_count[6]={0,0,0,0,0,0};//各轴的Pi
  int step_completed=0;//步数计数器
  int PUL_Up_flag[6]={0,0,0,0,0,0};//各轴脉冲是否置高标志位

  // 初始化所有电机
  for (int i=0;i<6;i++)
  {
    Set_Motor_Dri(MOTOR_dir_array[i], i);     //为每个电机设定方向
    digitalWrite(MOTOR_ENA_ARRAY[i], LOW);    //为每个机使能
    deta_step[i] = MOTOR_step_array[i];

    }
  
  step_even_count = max(deta_step[0],deta_step[1]);
  step_even_count = max(deta_step[2],step_even_count);
  step_even_count = max(deta_step[3],step_even_count); 
  step_even_count = max(deta_step[4],step_even_count); 
  step_even_count = max(deta_step[5],step_even_count); //取各轴最大值作为X（ΔX）
  
  step_count[0] = -(step_even_count>>1);                //-ΔX/2  ΔX➗2，后边所有数据都是除2处理 
  step_count[1] = step_count[0];
  step_count[2] = step_count[0];
  step_count[3] = step_count[0];
  step_count[4] = step_count[0];
  step_count[5] = step_count[0];
 
  // ΔX都是一致的
  // < 0 Pi + ΔY
  // > 0 Pi + ΔY - ΔX
  //并且应该注意到 P1 = ΔY - ΔX/2
  
  while(step_completed<step_even_count)
  {
      step_count[0]+=deta_step[0];  //计算小于0 的情况 = Pi + ΔY
      if(step_count[0]>0)    //Pi >0
      {
          digitalWrite(MOTOR_PUL_ARRAY[0], HIGH);
          step_count[0]-=step_even_count; //由于Pi + ΔY已经计算，此处直接减去ΔX即可
          PUL_Up_flag[0]=1;
          }
   
         step_count[1]+=deta_step[1];
        if(step_count[1]>0)
        {
          digitalWrite(MOTOR_PUL_ARRAY[1], HIGH);
          step_count[1]-=step_even_count;
          PUL_Up_flag[1]=1;
          }
   
         step_count[2]+=deta_step[2];
        if(step_count[2]>0)
        {
          digitalWrite(MOTOR_PUL_ARRAY[2], HIGH);
          step_count[2]-=step_even_count;
          PUL_Up_flag[2]=1;
          }
  
        step_count[3]+=deta_step[3];
        if(step_count[3]>0)
        {
          digitalWrite(MOTOR_PUL_ARRAY[3], HIGH);
          step_count[3]-=step_even_count;
          PUL_Up_flag[3]=1;
         }
  
        step_count[4]+=deta_step[4];
        if(step_count[4]>0)
        {
          digitalWrite(MOTOR_PUL_ARRAY[4], HIGH);
          step_count[4]-=step_even_count;
          PUL_Up_flag[4]=1;
         }
  
        step_count[5]+=deta_step[5];
        if(step_count[5]>0)
        {
          digitalWrite(MOTOR_PUL_ARRAY[5], HIGH);
          step_count[5]-=step_even_count;
          PUL_Up_flag[5]=1;
         }

       // 按最短脉冲间隔发送信号
       delayMicroseconds(MOTOR_speedtime_array[0]);//脉冲间隔

       // 为所有电机产生脉冲下降沿
       for (int i=0;i<6;i++)  //根据标志位对步进电机脉冲口置低电平，形成脉冲
       {
          if (PUL_Up_flag[i]==1)
          {
            digitalWrite(MOTOR_PUL_ARRAY[i], LOW);
            PUL_Up_flag[i]=0;
            }

        }
      step_completed+=1;
       
    }

    
  for (int i=0;i<6;i++)
  {
   digitalWrite(MOTOR_ENA_ARRAY[i], HIGH);//电机禁能
  }
}
/************************************************************/
void show_Cmd()//打印当前收到的命令
{
  for (int i=0;i<CMD_LENGTH;i++)
    {
      Serial.print(Cmd_string[i]);                      // 输出信息
      }
    Serial.println(" ");
}
/************************************************************/
char Driextract(char chstring[CMD_LENGTH])              //提取运动角度
 {
  char dir = 0 ;
  if (chstring[3]=='+')
  {
    dir=UP;
    }
    else if(chstring[3]=='-')
      {
    dir=DOWN;
    }
   else if(chstring[3]=='0')
      {
    dir=FREE;
    }
   else if(chstring[3]=='2')
      {
    dir=STOP;
    }
  return dir;
  }

/************************************************************/
int Stepextract(char chstring[CMD_LENGTH])              //提取运动角度
 {
  int Step=0;
    for (int i=4;i<9;i++)
  {
    chstring[i]=chstring[i]-48;                         //由ASCII码转化为真实数值
    }
  Step= (chstring[4]*10000+chstring[5]*1000+chstring[6]*100+chstring[7]*10+chstring[8]*1);//计算命令中的电机步数
  return Step;
  }
/************************************************************/
int Speedextract(char chstring[CMD_LENGTH])              //提取运动角度
 {
  int Speed=0;
    for (int i=10;i<14;i++)
    {
    chstring[i]=chstring[i]-48;                         //由ASCII码转化为真实数值
    }
  Speed = (chstring[10]*1000+chstring[11]*100+chstring[12]*10+chstring[13]*1);//计算命令中的电机步数
  return Speed;
  }
/************************************************************/
/* drill motor control function                             */
/* Speed: 0~255,PWM调速                                      */
/* Dri 1：正转；-1：反转；0：停止；1：刹车                        */
/* MOTOR_Num:0~1                                            */
/************************************************************/
// 直流电机控制
void Drill_Motor_Run( char Speed, int Dri, char MOTOR_Num)
{
  digitalWrite(DRILL_MOTOR_STBY, HIGH);//直流电机模块使能
  // 设置速度（PWM）
  analogWrite(DRILL_MOTOR_PWM_ARRAY[MOTOR_Num], Speed);//电机PWM调速

  // 设置方向
  if (Dri==UP)  //正转
  {
     digitalWrite(DRILL_MOTOR_IN1_ARRAY[MOTOR_Num], HIGH);//电机停止
     digitalWrite(DRILL_MOTOR_IN2_ARRAY[MOTOR_Num], LOW);//电机停止
    }
  else if(Dri==DOWN)  //正转
  {
     digitalWrite(DRILL_MOTOR_IN1_ARRAY[MOTOR_Num], LOW);//电机停止
     digitalWrite(DRILL_MOTOR_IN2_ARRAY[MOTOR_Num], HIGH);//电机停止
      }
  else if(Dri==FREE)  //电机放松 / 停止
  {
     digitalWrite(DRILL_MOTOR_IN1_ARRAY[MOTOR_Num], LOW);//电机停止
     digitalWrite(DRILL_MOTOR_IN2_ARRAY[MOTOR_Num], LOW);//电机停止
  }
  else if(Dri==STOP)  //电机刹车
  {
     digitalWrite(DRILL_MOTOR_IN1_ARRAY[MOTOR_Num], HIGH);//电机停止
     digitalWrite(DRILL_MOTOR_IN2_ARRAY[MOTOR_Num], HIGH);//电机停止
  }
  else
  {
    ;
    }
}

/************************************************************/
// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  digitalWrite(led2, HIGH);   // 点亮LED
    digitalWrite(led3, LOW);   // 点亮LED
      digitalWrite(led4, HIGH);   // 点亮LED
    for(int i=0;i<TOTAL_MOTOR_NUM;i++)  //电机控制IO使能
    {
      pinMode(MOTOR_PUL_ARRAY[i], OUTPUT);
      pinMode(MOTOR_DIR_ARRAY[i], OUTPUT);
      pinMode(MOTOR_ENA_ARRAY[i], OUTPUT);
  
      digitalWrite(MOTOR_ENA_ARRAY[i], HIGH);//电机禁能
      digitalWrite(MOTOR_DIR_ARRAY[i], LOW);//设置默认方向
      }

    
    pinMode(DRILL_MOTOR_STBY, OUTPUT);  //直流电机芯片使能引脚
    for(int i=0;i<TOTAL_DRILL_MOTOR_NUM;i++)  //电机控制IO使能
    {
      //pinMode(DRILL_MOTOR_PWM_ARRAY[i], OUTPUT);
      pinMode(DRILL_MOTOR_IN1_ARRAY[i], OUTPUT);
      pinMode(DRILL_MOTOR_IN2_ARRAY[i], OUTPUT);
      
      //analogWrite(DRILL_MOTOR_PWM_ARRAY[i], 0);//电机禁能
      digitalWrite(DRILL_MOTOR_IN1_ARRAY[i], LOW);//电机停止
      digitalWrite(DRILL_MOTOR_IN2_ARRAY[i], LOW);//电机停止
    }
    digitalWrite(DRILL_MOTOR_STBY, LOW);//直流电机模块使能
  
    Serial.begin(9600);
    Serial.println("Serial begin ok!");
}

void loop() {
    
//    MA0+05000S0050
    
    Serial.readBytes(Cmd_string, CMD_LENGTH);  //从串口读取命令
    
    if(Cmd_string[0]=='M')  //如果是MOVE指令（步进电机命令）
    { 
      if(Cmd_string[1]=='A')  // 单轴控制 "MA0+05000S0050"    
      {
        //show_Cmd();
        char MOTOR_num=Cmd_string[2]-48;    //计算当前要操作的电机序号——电机编号
        char MOTOR_dir=Driextract(Cmd_string);  // 方向
        int MOTOR_step=Stepextract(Cmd_string);  // 步数
        int MOTOR_speedtime=Speedextract(Cmd_string);  // 速度
        Motor_Move_count( MOTOR_step, MOTOR_dir, MOTOR_num, MOTOR_speedtime);  //开始运动
        Serial.println("MA MOVE ok!"); 
        
        for (int i=0;i<CMD_LENGTH;i++)
        {
          Cmd_string[i]=0;                      // 清除命令数组
          }
       }

     if(Cmd_string[1]=='T')  // XY轴控制 "MTX+05000S0050"   
                             // 类似处理，但同步控制两个电机
      {
        //show_Cmd();
        char MOTOR_num=Cmd_string[2];         //读取电机序号
        char MOTOR_dir=Driextract(Cmd_string);//读取命令方向
        int MOTOR_step=Stepextract(Cmd_string);
        int MOTOR_speedtime=Speedextract(Cmd_string);
        Robot_Move_XY_count( MOTOR_step, MOTOR_dir, MOTOR_num, MOTOR_speedtime);  //开始运动
        Serial.println("MT MOVE ok!"); 
        
        for (int i=0;i<CMD_LENGTH;i++)
        {
          Cmd_string[i]=0;                      // 清除命令数组
          }
       }

      if(Cmd_string[1]=='M')  // 多轴联动 "MM0+05000S0050"
      {
        //show_Cmd();

        if (CMD_Count_flag==Cmd_string[2]-48)  //设计原则：要求多轴联动时多电机命令必须按电机序号连续发送
        {
            MOTOR_num_array[CMD_Count_flag]       = Cmd_string[2]-48;         //读取电机序号
            MOTOR_dir_array[CMD_Count_flag]       = Driextract(Cmd_string);//读取命令方向
            MOTOR_step_array[CMD_Count_flag]      = Stepextract(Cmd_string);
            MOTOR_speedtime_array[CMD_Count_flag] = Speedextract(Cmd_string);
            
            for (int i=0;i<CMD_LENGTH;i++)
            {
              Cmd_string[i]=0;                      // 清除命令数组
            }
            
            CMD_Count_flag = CMD_Count_flag+1;  //命令计数加1
            
            if (CMD_Count_flag==6)        // 需要接收6条命令（0-5号电机）
                                          // 当收到第6条命令时执行多轴联动
            
            {
              Robot_Move_Multi_axis_count( MOTOR_dir_array, MOTOR_step_array, MOTOR_speedtime_array);
              Serial.println("MM MOVE ok!"); 
              CMD_Count_flag=0;
             }
          }
          else
          {
           for (int i=0;i<CMD_LENGTH;i++)
              {
                Cmd_string[i]=0;                      // 清除命令数组
              }
           }
      }
    }
    
    
    if(Cmd_string[0]=='D')  //直流电机转动控制：直流电机命令 "DR0+00000S0100"
    { 
      if(Cmd_string[1]=='R')     
      {
        char Drill_MOTOR_num=Cmd_string[2]-48;    //计算当前要操作的电机序号
        char Drill_MOTOR_dir=Driextract(Cmd_string);
        char Drill_MOTOR_speed=char(Speedextract(Cmd_string)); 
        Drill_Motor_Run( Drill_MOTOR_speed, Drill_MOTOR_dir, Drill_MOTOR_num);

        for (int i=0;i<CMD_LENGTH;i++)
        {
          Cmd_string[i]=0;                      // 清除命令数组
        }
      }
    }
}
