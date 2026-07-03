#include "main.h"
#include "math.h"
#include "MatrixKeypad.h"

static const char keypadMap[NUM_ROWS][NUM_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

/**
  * @brief  Initializes the keypad handle structure with the specified row and column mappings.
  *         This function sets the initial states of the variables, such as clearing the raw key,
  *         stable key, debounce counter, and resetting the scan timer.
  * @param  hkeypad Pointer to a Keypad_HandleTypeDef structure that contains
  *         the configuration and state information for the keypad.
  * @param  rows Pointer to an array of GPIO_PortMap_HandleTypeDef structures containing Row pin mappings.
  * @param  cols Pointer to an array of GPIO_PortMap_HandleTypeDef structures containing Column pin mappings.
  * @retval None
  */
void Keypad_Init(Keypad_HandleTypeDef *hkeypad, GPIO_PortMap_HandleTypeDef *rows, GPIO_PortMap_HandleTypeDef *cols)
{
    hkeypad->Rows = rows;
    hkeypad->Cols = cols;
    hkeypad->_lastRawKey = '\0';
    hkeypad->_stableKey = '\0';
    hkeypad->_debounceCounter = 0;
    hkeypad->_lastScanTime = 0;
}

/**
  * @brief  Scans the physical matrix keypad to check if a key is currently pressed.
  *         It sequentially drives each row low (GND) and reads the columns. If a column is read as low,
  *         the corresponding key from the map is returned. Before returning, the row is restored to its
  *         inactive high state.
  * @param  hkeypad Pointer to a Keypad_HandleTypeDef structure that contains
  *         the configuration and state information for the keypad.
  * @retval char The character of the pressed key, or '\0' if no key is detected.
  */
char Keypad_Scan(Keypad_HandleTypeDef *hkeypad)
{
    // Asegurar que todas las filas estén inactivas (Open-Drain HIGH / flotante)
    for (int r = 0; r < NUM_ROWS; r++) 
    {
        HAL_GPIO_WritePin(hkeypad->Rows[r].Port, hkeypad->Rows[r].Pin, GPIO_PIN_SET);
    }

    for (int r = 0; r < NUM_ROWS; r++) 
    {
        // Activar la fila actual (poner a LOW / GND)
        HAL_GPIO_WritePin(hkeypad->Rows[r].Port, hkeypad->Rows[r].Pin, GPIO_PIN_RESET);

        // Buscar en las columnas
        for (int c = 0; c < NUM_COLS; c++) 
        {
            if (HAL_GPIO_ReadPin(hkeypad->Cols[c].Port, hkeypad->Cols[c].Pin) == GPIO_PIN_RESET) 
            {
                // Restaurar la fila a inactiva antes de retornar la tecla
                HAL_GPIO_WritePin(hkeypad->Rows[r].Port, hkeypad->Rows[r].Pin, GPIO_PIN_SET);
                return keypadMap[r][c];
            }
        }
        // Desactivar la fila actual
        HAL_GPIO_WritePin(hkeypad->Rows[r].Port, hkeypad->Rows[r].Pin, GPIO_PIN_SET);
    }
    return '\0';
}

/**
  * @brief  Updates the keypad state machine and handles non-blocking debouncing.
  *         It samples the matrix keypad periodically every 20 milliseconds. If the key state is
  *         stable for at least two consecutive scans (40 ms), it registers the stable key and
  *         triggers the Keypad_OnkeyPress callback function.
  * @param  hkeypad Pointer to a Keypad_HandleTypeDef structure that contains
  *         the configuration and state information for the keypad.
  * @retval None
  */
void Keypad_Update(Keypad_HandleTypeDef *hkeypad)
{
    uint32_t currentTime = HAL_GetTick();
    if (currentTime - hkeypad->_lastScanTime >= 20) 
    { // Muestreo cada 20 ms
        hkeypad->_lastScanTime = currentTime;
        char currentRawKey = Keypad_Scan(hkeypad);
        if (currentRawKey == hkeypad->_lastRawKey) 
        {
            if (currentRawKey != '\0') 
            {
                hkeypad->_debounceCounter++;
                if (hkeypad->_debounceCounter >= 2) 
                {
                    if (currentRawKey != hkeypad->_stableKey) 
                    {
                        hkeypad->_stableKey = currentRawKey;
                        Keypad_OnkeyPress(hkeypad->_stableKey);
                    }
                }
            } 
            else 
            {
                hkeypad->_debounceCounter = 0;
                hkeypad->_stableKey = '\0';
            }
        } 
        else 
        {
            hkeypad->_lastRawKey = currentRawKey;
            hkeypad->_debounceCounter = 0;
        }
    }
}