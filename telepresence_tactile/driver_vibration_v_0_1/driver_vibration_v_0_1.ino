// The University of Sheffield
// Sheffield Robotics Laboratory
// Vibration motor driver version 0.1
// Author: Uriel Martinez
// date: 08-04-2015


#define MAX_SIZE_COMMAND 20
#define MAX_NUM_PARAMETERS 20

int count = 0;
char command[MAX_SIZE_COMMAND];
char commands_char[MAX_NUM_PARAMETERS][MAX_SIZE_COMMAND];
int ncommand = 0;
char current_char;
bool commandStatus = false;
bool clearBuffer = true;
int maxMotors = 12;

void setup()
{
    Serial.begin(9600);
}


void loop()
{
    if( Serial.available() > 0 )
    {
        if( clearBuffer == true )
        {
            for( int i = 0; i < MAX_NUM_PARAMETERS; i++ )
            {
                for( int j = 0; j < MAX_SIZE_COMMAND; j++ )
                    commands_char[i][j] = '\0';
            }

            count = 0;
            ncommand = 0;
            clearBuffer = false;            
        }

        current_char = Serial.read();

        if( current_char != '\r' )
        {
            if( current_char != ' ' )
            {
                commands_char[ncommand][count] = current_char;
                count++;
            }
            else
            {
                commands_char[ncommand][count] = '\0';
                count = 0;
                ncommand++;       
            }
        }
        else
        {
            commandStatus = commandList(commands_char[0]);
            replyAcknowledge(commandStatus);
        
            if( commandStatus == true )
                replyAcknowledge(executeCommand(commands_char));
            
            clearBuffer = true;
        }
    }
}    

/* Function for execution of commands */
bool executeCommand(char cmdReceived[][MAX_SIZE_COMMAND])
{
    int duty_cycle_int[20];

    if( ncommand < maxMotors )
    {
      // right hand
      analogWrite(2, 0);
      analogWrite(3, 0);
      analogWrite(4, 0);
      analogWrite(5, 0);
      analogWrite(6, 0);
      analogWrite(7, 0);
      // left hand
      analogWrite(8, 0);
      analogWrite(9, 0);
      analogWrite(10, 0);
      analogWrite(11, 0);
      analogWrite(12, 0);
      analogWrite(13, 0);

      return false;
    }
    else
    {
      // right hand
      duty_cycle_int[0] = atoi(cmdReceived[1]);
      duty_cycle_int[1] = atoi(cmdReceived[2]);
      duty_cycle_int[2] = atoi(cmdReceived[3]);
      duty_cycle_int[3] = atoi(cmdReceived[4]);
      duty_cycle_int[4] = atoi(cmdReceived[5]);
      duty_cycle_int[5] = atoi(cmdReceived[6]);
      // left hand
      duty_cycle_int[6] = atoi(cmdReceived[7]);
      duty_cycle_int[7] = atoi(cmdReceived[8]);
      duty_cycle_int[8] = atoi(cmdReceived[9]);
      duty_cycle_int[9] = atoi(cmdReceived[10]);
      duty_cycle_int[10] = atoi(cmdReceived[11]);
      duty_cycle_int[11] = atoi(cmdReceived[12]);

      // right hand
      analogWrite(2, duty_cycle_int[0]);
      analogWrite(3, duty_cycle_int[1]);
      analogWrite(4, duty_cycle_int[2]);
      analogWrite(5, duty_cycle_int[3]);
      analogWrite(6, duty_cycle_int[4]);
      analogWrite(7, duty_cycle_int[5]);
      // left hand
      analogWrite(8, duty_cycle_int[6]);
      analogWrite(9, duty_cycle_int[7]);
      analogWrite(10, duty_cycle_int[8]);
      analogWrite(11, duty_cycle_int[9]);
      analogWrite(12, duty_cycle_int[10]);
      analogWrite(13, duty_cycle_int[11]);

      return true;
    }
}

/* Send reply ACK/NACK to client */
void replyAcknowledge(bool cmdStatus)
{
//    delay(100);
    if( cmdStatus == true )
        sendACK();
    else
        sendNACK();

    Serial.flush();
}

/* Print ACK message */
void sendACK()
{
    Serial.print("ACK\n");
}

/* Print NACK message */
void sendNACK()
{
    Serial.print("NACK\n");
}

/* Check the command received */
bool commandList(char *cmdReceived)
{
    char *commandArray[] = {"@SETPWM","@SETOFF","@SETON"};
    int ncommands = 3;
    
    for( int i = 0; i < ncommands; i++ )
    {
        if( !strcmp(commandArray[i], cmdReceived) )
        {
            return true;
        }
    }
    
    return false;
}

