#include "main.h"

int initialized=0;
RNG_HandleTypeDef hrng;

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */
  initialized = 1;
  /* USER CODE END RNG_Init 2 */

}


uint32_t rand_range(uint32_t a, uint32_t b) {
	if(!initialized)
	{
		HAL_Delay(100);
		CDC_Transmit_HS("Random not initialized!\n", strlen("Random not initialized!\n"));
		return -1;
	}

	uint32_t rand = 0;
	uint32_t MAX = 4294967295;
	if(b>a && HAL_RNG_GenerateRandomNumber(&hrng, &rand) == HAL_OK)
	{
		return a+rand/(MAX/(b-a));
	}
	else
	{
		HAL_Delay(100);
		CDC_Transmit_HS("rng error\n", strlen("rng error\n"));
	}
	return -1;
}
