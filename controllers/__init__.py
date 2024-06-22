if __name__ == "__main__":
    from bme688_receiver import BME688_Receiver
    from bmi270_receiver import BMI270_Receiver
    from ui_controller import UI_Controller
else:
    from controllers.bme688_receiver import BME688_Receiver
    from controllers.bmi270_receiver import BMI270_Receiver
    from controllers.ui_controller import UI_Controller