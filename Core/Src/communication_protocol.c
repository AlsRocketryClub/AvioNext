#include "communication_protocol.h"
#include "LoRA.h"
#include "usb_device.h"

void communicationHandler(void reliableReceiveHandle(char*), void streamReceiveHandle(char*), char* streamSendHandle(int), struct ReliableSendConfig reliableSendHandle(), int initial_communication_state) {
  //just for testing
  char sendMessage[MAX_PKT_LENGTH];
  char state[MAX_PKT_LENGTH] = "";
  char response_packet[MAX_PKT_LENGTH] = "";
  //from here it's okay
  int communication_state = initial_communication_state;
  int max_packet_count = 0;
  char previous_packet[MAX_PKT_LENGTH] = "";
  char received_packet[MAX_PKT_LENGTH] = "";
  char general_buffer[MAX_PKT_LENGTH] = "";
  uint32_t previousTime = HAL_GetTick();
  uint32_t wait_time = rand_range(3, 13)*100;

  while (1) {
      if(communication_state == RECEIVING_RELIABLE)
      {
        if(recv_packet(received_packet, MAX_PKT_LENGTH))
        {
          previousTime = HAL_GetTick();

          if(sscanf(received_packet, "$ %s", state) == 1)
          {
            communication_state = SENDING_RELIABLE;
          }
          else if(sscanf(received_packet, "! %d", &max_packet_count) == 1)
          {
            communication_state = SENDING_STREAM;
          }
          else if(strcmp(received_packet, previous_packet)==0)
          {
            //send acknowledge again
            LoRA_sendPacket(received_packet);
          }
          else
          {
        	//send acknowledge
            strcpy(previous_packet, received_packet);
            LoRA_sendPacket(received_packet);

            reliableReceiveHandle(received_packet);

          }
        }
      }
      else if(communication_state == RECEIVING_STREAM)
      {
        if(recv_packet(received_packet, MAX_PKT_LENGTH))
        {
          previousTime = HAL_GetTick();
          if(sscanf(received_packet, "$ %s", state) == 1)
          {
            communication_state = SENDING_RELIABLE;
          }
          else
          {
        	streamReceiveHandle(received_packet);
          }
        }
        else if(HAL_GetTick()-previousTime > wait_time)
        {
          wait_time = rand_range(3, 13)*100;
          previousTime = HAL_GetTick();
          //give up SENDING
          sprintf(general_buffer, "! %d", max_packet_count);
          LoRA_sendPacket(general_buffer);
        }
      }
      else if(communication_state == SENDING_STREAM)
      {
        if(max_packet_count == 0)
        {
          communication_state = TRANSITIONING;
          LoRA_sendPacket("$");
        }
        else
        {
          //send whatever
          char* msg = streamSendHandle(max_packet_count);
          LoRA_sendPacket(msg);
          max_packet_count--;
        }

      }
      else if(communication_state == SENDING_RELIABLE)
      {
        reliable_send_packet("*");
        struct ReliableSendConfig config = reliableSendHandle();
        for(int i = 0; i < config.messages_count; i++)
        {
        	if(config.messages[i][0] == '$' || config.messages[i][0] == '*' || config.messages[i][0] == '!')
        	{
        		HAL_Delay(100);
        		CDC_Transmit_HS("Can't send control commands! ('$', '*', '!')\n", strlen("Can't send control commands! ('$', '*', '!')\n"));
        	}
        	else
        	{
        		reliable_send_packet(config.messages[i]);
        	}
        }
        if(config.mode==RECEIVING_STREAM)
        {
          communication_state = RECEIVING_STREAM;
          max_packet_count = config.streamable_packets;
          //to do rename this:
          sprintf(sendMessage, "! %d", max_packet_count);
          LoRA_sendPacket(sendMessage);
        }
        else if(config.mode==TRANSITIONING)
        {
          communication_state = TRANSITIONING;
          LoRA_sendPacket("$");
        }
        else
        {
            HAL_Delay(100);
        	CDC_Transmit_HS("Shouldn't try to transition to this mode.\n", strlen("Shouldn't try to transition to this mode.\n"));
        }
      }
      else if(communication_state == TRANSITIONING)
      {

          if(recv_packet(received_packet, MAX_PKT_LENGTH))
          {
            previousTime = HAL_GetTick();
            if(strcmp(received_packet, "*")==0)
            {
              strcpy(previous_packet, received_packet);
              communication_state = RECEIVING_RELIABLE;
              LoRA_sendPacket(received_packet);
            }
          }
          else if (HAL_GetTick()-previousTime > wait_time)
          {
            wait_time = rand_range(3, 13)*100;
            previousTime = HAL_GetTick();
            LoRA_sendPacket("$");
          }
      }
  }
}
