/*
// Опрос сенсоров
void sensor_Report() {
  //Serial.println("Start reporting...");
  char report[64];

  uint8_t NewDataReady = 0;
  int statusS;
  angle = as5600.readAngle();
  dataStr = "-----------------------------------------------";
  Serial.println(dataStr);
  Serial1.println(dataStr);

  float temp = 25 + ((float)(analogRead(ATEMP) * 3200) / 4096 - 760) / 2.5;
  uint8_t hour, minute, second, day, month, year, weekDay;
  rtc.getTime(&hour, &minute, &second, 0, nullptr);
  rtc.getDate(&weekDay, &day, &month, &year);
  snprintf(report, sizeof(report), "Time:%02d:%02d:%02d, Date:%02d/%02d/%02d", hour, minute, second, day, month, year);
  dataStr = String(report);
  dataStr += "  Temperature = " + String(temp);
  Serial.println(dataStr);
  Serial1.println(dataStr);

  snprintf(report, sizeof(report), "Angle = %d | Lenght = %d | position = %d ", angle, channelLength, currentPosition);
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = "Batt voltage = " + String(batteryVoltage) + "V Charge = " + String(batteryCharge) + "% limit = " + String(minBattCharge) + "%";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  dataStr = "Inverse = ";
  if (inverse) dataStr += "YES   FIFO_LIFO = ";
  else dataStr += "NO   FIFO_LIFO = ";
  if (fifoLifo) dataStr += "LIFO";
  else dataStr += "FIFO";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  snprintf(report, sizeof(report), "Forwrd dist = %d | Revrs dist = %d ", distance[1], distance[0]);
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  snprintf(report, sizeof(report), "Forwrd plt dist = %d | Revrs plt dist = %d ", distance[3], distance[2]);
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  detect_Pallete();
  snprintf(report, sizeof(report), "Plt dtchk F1 = %d | Plt dtchk F2 = %d ", detectPalleteF1, detectPalleteF2);
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  snprintf(report, sizeof(report), "Plt dtchk R1 = %d | Plt dtchk R2 = %d ", detectPalleteR1, detectPalleteR2);
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  dataStr = "In channel: ";
  if (digitalRead(CHANNEL)) dataStr += "YES";
  else dataStr += "NO";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  dataStr = "Lifter UP: ";
  if (digitalRead(DL_UP)) dataStr += "NO | Lifter DOWN: ";
  else dataStr += "YES | Lifter DOWN: ";
  if (!digitalRead(DL_DOWN)) dataStr += "YES";
  else dataStr += "NO";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  snprintf(report, sizeof(report), "Bumper forward = %d | Bumper reverse = %d ", digitalRead(BUMPER_F), digitalRead(BUMPER_R));
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = "MPR = " + String(interPalleteDistance) + " max speed = " + String(maxSpeed);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = "Wait time on unload = " + String(waitTime / 1000) + " Sec";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  dataStr = "Zero point MPR = " + String(mprOffset) + " channel offset = " + String(chnlOffset);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  if (sizeof(shuttleStatus) > status) {
    dataStr = "Status = " + String(shuttleStatus[status]) + "  (" + String(status) + ")";
    Serial.println(dataStr);
    Serial1.println(dataStr);
  } else {
    dataStr = "Shuttle status num = " + String(status);
    Serial.println(dataStr);
    Serial1.println(dataStr);
  }
  delay(20);
  Serial.println("Shuttle number = " + String(shuttleNum + 1) + " Shuttle length = " + String(shuttleLength));
  Serial1.println("Shuttle number = " + String(shuttleNum + 1) + " Shuttle length = " + String(shuttleLength));
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = "Weehl diameter = " + String(weelDia);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = "-----------------------------------------------";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  Serial1.println("CB " + String(batteryCharge));
  if (!errorStatus[0] && (batteryCharge > 0 && batteryCharge <= minBattCharge)) {
    lifter_Down();
    moove_Forward();
    add_Error(11);
    status = 5;
    return;
  }
  Serial1.println("CV " + String(batteryVoltage));
  //makeReport();
}

// Формирование бинарного репорта (на замену текстового)
void makeReport() {
  reportData.shuttleNumber = shuttleNum;
  reportData.shuttleLength = shuttleLength;
  reportData.maxSpeed = maxSpeed;
  reportData.minSpeed = minSpeed;
  reportData.interPalleteDistance = interPalleteDistance;
  reportData.inverse = inverse;
  reportData.fifoLifo = fifoLifo;
  reportData.lifterSpeed = lifter_Speed;
  reportData.waitTime = waitTime;

  batteryData.minBattCharge = minBattCharge;
  batteryData.batteryVoltage = batteryVoltage;
  batteryData.batteryCharge = batteryCharge;
  reportData.battery = batteryData;

  memcpy(&reportData.calibrateEncoder_F, calibrateEncoder_F, 8);
  memcpy(&reportData.calibrateEncoder_R, calibrateEncoder_R, 8);
  memcpy(&reportData.calibrateSensor_F, calibrateSensor_F, 3);
  memcpy(&reportData.calibrateSensor_R, calibrateSensor_R, 3);
  reportData.timingBudget = timingBudget;
  reportData.mprOffset = mprOffset;
  reportData.chnlOffset = chnlOffset;
  reportData.blinkTime = blinkTime;

  eepromStat.load = loadCounter;
  eepromStat.unload = unloadCounter;
  eepromStat.compact = compact;
  eepromStat.liftUp = liftUpCounter;
  eepromStat.liftDown = liftDownCounter;
  eepromStat.totalDist = totalDist;
  memcpy(&reportData.shuttleStats, &eepromStat, sizeof(eepromStat));
  memcpy(&reportData.dateTime, &globalDateTime, sizeof(globalDateTime));
  reportData.temp = temp;

  memcpy(&reportData.errorStatus, errorStatus, 16);

  uint16_t dataSize = sizeof(reportData);

  Serial1.write(0xFF);
  Serial1.write(0xFE);
  Serial1.write((const uint8_t*)&dataSize, sizeof(dataSize));
  Serial1.write((const char*)&reportData, dataSize);
  Serial1.write(0xFE);
  Serial1.write(0xFF);
  Serial1.write('\n');
}

*/

void blink_Error() {
  static int countError = millis();
  static uint8_t errCounter = 0;
  if (millis() - countError < 200) return;
  if (errCounter == 0 || errCounter == 2 || errCounter == 4 || errCounter == 6 || errCounter == 8) {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    errCounter++;
  }
  else if (errCounter == 1 || errCounter == 3 || errCounter == 5 || errCounter == 7) {
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BOARD_LED, LOW);
    errCounter++;
  }
  else if (errCounter == 9) {
    digitalWrite(GREEN_LED, LOW);
    errCounter++;
    IWatchdog.reload();
    motor_Stop();
    Can1.read(CAN_RX_msg);
  }
  else if (errCounter == 14) {
    debuger++;    
    if (debuger == 1) {
      uint8_t i = 0;
      while (errorStatus[i]) {
        if (batteryCharge > minBattCharge && errorStatus[i] == 11) errorStatus[i] = 0;
        dataStr += shuttleErrors[errorStatus[i]] + " ";
        i++;
      }
      makeLog(LOG_ERROR, "Shuttle ERROR! Code: %04X", errorCode);
      sensor_Report();
    }
    // if (debuger == 3 || debuger == 7) {
    //   Serial1.println("CB " + String(batteryCharge));
    // }
    // if (debuger == 2 || debuger == 6) {
    //   Serial1.println("CV " + String(batteryVoltage));
    // }
    else if (debuger == 10) debuger = 0;    
    status = 16;
    send_Cmd();
    status = 19;
    errCounter = 0;
  }
  else errCounter++;
  countError = millis();
}

void read_BatteryCharge() {
  while (Serial3.available()) Serial3.read();
  uint8_t datab[7] = { 0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77 };
  if (!digitalRead(RS485)) pinMode(RS485, OUTPUT);
  else {
    makeLog(LOG_ERROR, "RS485 collision...");
    return;
  }
  digitalWrite(RS485, HIGH);
  delay(1);
  Serial3.write(datab, 7);
  delay(20);
  digitalWrite(RS485, LOW);
  delay(20);
  uint8_t dataRead[50] = { 0 };
  uint8_t data = 0;
  uint8_t i = 0;
  int cnt = millis();
  //Serial.print("Batt data: ");
  //Serial1.print("Batt data: ");
  while (Serial3.available())  // Получаем команду
  {
    if (status == 5) break;
    dataRead[i] = Serial3.read();
    //Serial.print(" " + String(dataRead[i], HEX));
    //Serial1.print(" " + String(dataRead[i], HEX));
    data = 1;
    delayMicroseconds(1750);
    i++;
    if (i > 5 && i == dataRead[3] + 6) {
      dataRead[i] = Serial3.read();
      break;
    }
    if (millis() - cnt > 100) {
      data = 0;
      break;
    }
  }
  /*if (data) {
    Serial.print(" " + String(dataRead[i], HEX));
    Serial.println(" .");
  }
  else Serial.println(" No data...");
  Serial1.print(" " + String(dataRead[i], HEX));
  Serial1.println(" .");*/
  //i--;
  if (data && dataRead[0] == 0xDD) {
    float volt = (float)(dataRead[4] * 256 + dataRead[5]) / 100;
    float current = (float)(dataRead[6] * 256 + dataRead[7]);
    if (current > 32768) current = current - 65536;
    current /= 100;
    float capacity = (float)(dataRead[8] * 256 + dataRead[9]) / 100;
    //if (batteryCharge - lrint(capacity / 0.3) < 3 && lrint(capacity / 0.3) <= 100) batteryCharge = lrint(capacity / 0.3);
    uint16_t sum = 0;
    uint16_t res = dataRead[i - 2] * 256 + dataRead[i - 1];
    for (uint8_t j = 3; j < i - 2; j++) sum += dataRead[j];
    sum = ~sum + 1;
    if (sum == res) {
      if (volt > 41 && volt < 60) batteryVoltage = volt;
      if ((batteryCharge - dataRead[23] < 3 && dataRead[23] - batteryCharge < 5) || batteryCharge == 0) {
        batteryCharge = dataRead[23];
        oldBattCharge = batteryCharge;
        battCount = 0;
      } else if (oldBattCharge - dataRead[23] < 2 && dataRead[23] - oldBattCharge < 3) {
        battCount++;
        oldBattCharge = dataRead[23];
        if (battCount > 2) {
          batteryCharge = oldBattCharge;
          battCount = 0;
        }
      } else oldBattCharge = dataRead[23];
      //countBatt = millis();
    }

    if (millis() - countBatt > 10000 || true) {
      /*dataStr = "Read battery fail. Data string: ";
      for (uint8_t j = 0; j < i + 1; j++) dataStr += String(dataRead[j]) + " ";
      Serial.println(dataStr);
      Serial1.println(dataStr);*/
      dataStr = "Calc checksumm = " + String(sum) + "  ; BMS checksumm = " + String(res);
      //Serial.println(dataStr);
      //Serial1.println(dataStr);
      countBatt = millis();
    }
  }
  while (Serial3.available()) Serial3.read();
  pinMode(RS485, INPUT_PULLDOWN);
  if (batteryCharge < 20) Serial2.print(shuttleNums[shuttleNum] + "wc002!");
}

// Serial1.println("CB " + String(batteryCharge));
// Serial1.println("CV " + String(batteryVoltage));

// makeLog(LOG_INFO, "online...");
// delay(20);
// Serial1.println("CB " + String(batteryCharge));
// delay(20);
// Serial1.println("CV " + String(batteryVoltage));

/*
  inByte = 0;
  int8_t inBytes[30];
  uint8_t i = 0;
  inStr = "";
  data = 0;
  while (Serial1.available()) {  // Получаем команду от Lilygo (WiFi)
    inByte = Serial1.read();
    inBytes[i] = inByte;
    i++;
    char inChar = (char)inByte;
    inStr += inChar;
    data = 1;
    delayMicroseconds(1750);
    if (inStr.length() > 30) break;
  }
  if (data) {
    String tempStr = inStr.substring(0, 2);
    if (tempStr == shuttleNums[shuttleNum]) {
      countPult = millis();
      tempStr = inStr.substring(2, 8);
      if (tempStr == "dCharg" || inStr == "dCharg") send_Cmd();              // Коннект с пультом
      else if (tempStr == "dRight" || inStr == "dRight") { statusTmp = 1; }  // Движение вперед
      else if (tempStr == "dLeft_" || inStr == "dLeft_") {
        statusTmp = 2;
      }                                                                      // Движение назад
      else if (tempStr == "dUp___" || inStr == "dUp___") { statusTmp = 3; }  // Поднять платформу вверх
      else if (tempStr == "dDown_" || inStr == "dDown_") {                   // Опустить платформу
        statusTmp = 4;
      } else if (tempStr == "dStop_" || inStr == "dStop_") {                 // Остановка
        statusTmp = 5;
        diffPallete = 0;
      } else if (tempStr == "dLoad_" || inStr == "dLoad_") { statusTmp = 6; }  // Выгрузка паллеты
      else if (tempStr == "dUnld_" || inStr == "dUnld_") {                     // Загрузка паллеты
        statusTmp = 7;
      } else if (tempStr == "dClbr_" || inStr == "dClbr_") { statusTmp = 10; } // Калибровка
      else if (tempStr == "dDemo_" || inStr == "dDemo_") {                     // Демо режим
        statusTmp = 11;
      } else if (tempStr == "dGetQu" || inStr == "dGetQu") { statusTmp = 12; } // Подсчет паллет
      else if (tempStr == "dSaveC" || inStr == "dSaveC") {                     // Сохранение параметров на флэш память
        saveEEPROMData(eepromData);
      } else if (tempStr == "dComFo" || inStr == "dComFo") { statusTmp = 14; } // Уплотнение паллет вперед
      else if (tempStr == "dComBa" || inStr == "dComBa") {                     // Уплотнение паллет назад
        statusTmp = 15;
      } else if (tempStr == "dSpGet" || inStr == "dSpGet") {                    // Запрос параметров из настроек
        statusTmp = 16;
        send_Cmd();
      } else if (tempStr == "dDataP" || inStr == "dDataP") {                   // Запрос данных из отладки
        statusTmp = 17;
        send_Cmd();
      } else if (tempStr == "tError" || inStr == "tError") {                   // Запрос ошибок
        statusTmp = 19;
        send_Cmd();
      } else if (tempStr == "dEvOn_" || inStr == "dEvOn_") {                   // Включение режима эвакуации
        statusTmp = 20;
        evacuate = 1;
      }  else if (tempStr == "dLLoad" || inStr == "dLLoad") { statusTmp = 21; }// Продолжительная загрузка
      else if (tempStr == "dLUnld" || inStr == "dLUnld") {                     // Продолжительная выгрузка
        statusTmp = 22;
      } else if (tempStr == "dReset" || inStr == "dReset") { statusTmp = 24; } // Сброс ошибок
      else if (tempStr == "dManua" || inStr == "dManua") {                     // Ручной режим
        statusTmp = 25;
        countManual = millis();
        send_Cmd();
      } else if (tempStr == "dHome_" || inStr == "dHome_") { statusTmp = 27; } // В начало канала
      else if (tempStr == "dWaitT" || inStr == "dWaitT") {                     // Время ожидания при загрузке
        statusTmp = 29;
      } else if (tempStr == "dEvOff" || inStr == "dEvOff") { evacuate = 0; }   // Выключение режима эвакуации
      else if (tempStr == "ngPing" || inStr == "ngPing") { pingCount = millis();}   // Удержание движения
      else if (tempStr == "dLIFO_" || inStr == "dLIFO_") { fifoLifo = 0; }     // Установка режима LIFO
      else if (tempStr == "dFIFO_" || inStr == "dFIFO_") {                     // Установка режима FIFO
        fifoLifo = 1;
      } else if (tempStr == "dRevOn" || inStr == "dRevOn") {                   // Включение реверса
        inverse = 0;
        eepromData.inverse = inverse;
        currentPosition = channelLength - currentPosition - 800;
      } else if (tempStr == "dReOff" || inStr == "dReOff") {                   // Выключение реверса
        inverse = 1;
        eepromData.inverse = inverse;
        currentPosition = channelLength - currentPosition - 800;
      }
      tempStr = inStr.substring(2, 5);
      if (tempStr == "dNN" || inStr == "dNN") {  // Установка номера шаттла
        shuttleNum = inStr.substring(5, 8).toInt() - 1;
        eepromData.shuttleNum = shuttleNum;
      } else if (tempStr == "dQt" || inStr == "dQt") {  // Выгрузка заданного количества паллет
        UPQuant = inStr.substring(5, 8).toInt();
        statusTmp = 23;
      } else if (tempStr == "dDm" || inStr == "dDm") {  // Установка межпаллетного расстояния
        interPalleteDistance = inStr.substring(5, 8).toInt();
        eepromData.interPalleteDistance = interPalleteDistance;
      } else if (tempStr == "dSl" || inStr == "dSl") {  // Установка длинны шаттла
        shuttleLength = inStr.substring(5, 8).toInt() * 10;
        eepromData.shuttleLength = shuttleLength;
      } else if (tempStr == "dSp" || inStr == "dSp") {  // Установка максимальной скорости
        maxSpeed = inStr.substring(5, 8).toInt();
        if (maxSpeed > 96) maxSpeed = 96;
        if (maxSpeed < minSpeed) maxSpeed = minSpeed + 5;
        eepromData.maxSpeed = maxSpeed;
      } else if (tempStr == "dBc" || inStr == "dBc") {  // Установка минимального заряда батареи
        minBattCharge = inStr.substring(5, 8).toInt();
        if (minBattCharge > 50) minBattCharge = 50;
        if (minBattCharge < 0) minBattCharge = 0;
        eepromData.minBattCharge = minBattCharge;
      } else if (tempStr == "dMr" || inStr == "dMr") {  // Движение назад на заданное расстояние
        statusTmp = 8;
        mooveDistance = inStr.substring(5, 8).toInt() * 10;
      } else if (tempStr == "dMf" || inStr == "dMf") {  // Движение вперед на заданное расстояние
        statusTmp = 9;
        mooveDistance = inStr.substring(5, 8).toInt() * 10;
      } else if (tempStr == "dWt" || inStr == "dWt") {  // Время ожидания при выгрузке
        statusTmp = 0;
        waitTime = inStr.substring(5, 7).toInt() * 1000;
        eepromData.waitTime = waitTime;
      } else if (tempStr == "dMo" || inStr == "dMo") {  // Смещение МПР
        statusTmp = 0;
        mprOffset = (int8_t)inStr.substring(5, 8).toInt() - 100;
        eepromData.mprOffset = mprOffset;
      } else if (tempStr == "dMc" || inStr == "dMc") {  // Смещение конца канала
        statusTmp = 0;
        chnlOffset = (int8_t)inStr.substring(5, 8).toInt() - 100;
        eepromData.chnlOffset = chnlOffset;
      }
    } else if (tempStr == "GS" || inStr == "GS") {  // Запрос от индикатора батареи номера шаттла
      Serial1.print("GS " + String(shuttleNum));
      delay(5);
      Serial1.print("GS " + String(shuttleNum));
    } else if (tempStr == "DT" || inStr == "DT") {  // Установка текущей даты и времени
      String input = inStr.substring(3, 23);
      int hour, minute, second, day, month, year;
      if (sscanf(input.c_str(), "%d:%d:%d %d.%d.%d", &hour, &minute, &second, &day, &month, &year) == 6) {
        if (isValidDateTime(hour, minute, second, day, month, year)) {
          rtc.setTime(hour, minute, second);
          rtc.setDate(getWeekDay(day, month, year), day, month, year - 2000);
          Serial.println("RTC updated successfully.");
          globalDateTime.hours = hour;
          globalDateTime.minutes = minute;
          globalDateTime.seconds = second;
          globalDateTime.days = day;
          globalDateTime.months = month;
          globalDateTime.years = year;
        } else {
          Serial.println("Invalid date/time format or value.");
        }
      } else {
        Serial.println("Invalid format. Use: HH:MM:SS DD/MM/YYYY");
      }
    }
    tempStr = inStr.substring(0, 8);
    if (tempStr == "Firmware") {
      motor_Stop();
      jumpToBootloader();
    } else if (tempStr == "Reboot__") {
      makeLog(LOG_INFO, "Reboot system by external command...");
      delay(20);
      HAL_NVIC_SystemReset();
    }
  }
*/