
MOBILE_ROBOT_FOLDER = robots/mobile
ROBOT_FOLDER = robots/non-mobile
SENSOR_FOLDER = components/sensors

install_models: 
	ln -s $(PWD)/$(ROBOT_FOLDER)/Afma6.ttm $(VREP_ROOT_DIR)/models/$(ROBOT_FOLDER)/Afma6.ttm
	ln -s $(PWD)/$(ROBOT_FOLDER)/Viper850.ttm $(VREP_ROOT_DIR)/models/$(ROBOT_FOLDER)/Viper850.ttm
	ln -s $(PWD)/$(MOBILE_ROBOT_FOLDER)/Mikrokopter.ttm $(VREP_ROOT_DIR)/models/$(MOBILE_ROBOT_FOLDER)/Mikrokopter.ttm
	ln -s $(PWD)/$(SENSOR_FOLDER)/Imu.ttm $(VREP_ROOT_DIR)/models/$(SENSOR_FOLDER)/Imu.ttm
	ln -s $(PWD)/$(SENSOR_FOLDER)/Dragonfly2.ttm $(VREP_ROOT_DIR)/models/$(SENSOR_FOLDER)/Dragonfly2.ttm

clean:	
	rm $(VREP_ROOT_DIR)/models/$(ROBOT_FOLDER)/Afma6.ttm
	rm $(VREP_ROOT_DIR)/models/$(ROBOT_FOLDER)/Viper850.ttm
	rm $(VREP_ROOT_DIR)/models/$(MOBILE_ROBOT_FOLDER)/Mikrokopter.ttm
	rm $(VREP_ROOT_DIR)/models/$(SENSOR_FOLDER)/Imu.ttm
	rm $(VREP_ROOT_DIR)/models/$(SENSOR_FOLDER)/Dragonfly2.ttm

