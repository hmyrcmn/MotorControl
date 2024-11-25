import serial
import time

class ODriveController:
    # TODO: her komutun işlendiğinin kontorlunu sağla mesela w larda none dönoyor responcelar bunun sonucunda komut başarılı işşlenmişmi kontrol et

    def __init__(self, port="COM3", baudrate=2000000, debug=False):
        self.previous_torque = None
        self.last_velocity = 0.0
        self.threshold = 1
        self.debug = debug
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            if self.debug:
                print(f"ser name: {self.ser.name}")
                print(f"ser is open: {self.ser.is_open}")
                print(f"baudrate: {self.ser.baudrate}")
                print(f"stopbits: {self.ser.stopbits}")
        except serial.SerialException as e:
            print(f"Seri port açma hatası: {e}")
            self.ser = None

    def send_command(self, command):
        """ODrive'a komut gönder."""
        if self.ser is not None:
            command += "\n"
            if self.debug:
                print(f"Sending command: {command.strip()}")
            self.ser.write(command.encode())
            time.sleep(0.1)
            response = self.read_response()
            if self.debug:
                print(f"Response: {response}")
            return response
        else:
            print("Seri port açık değil, komut gönderilemiyor.")
            return None

    def read_response(self):
        """ODrive'dan gelen cevabı oku."""
        if self.ser is not None:
            time.sleep(0.1)
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode("utf-8").strip()
                return response
        return None

    def get_bus_voltage(self):
        """Vbus voltajını al."""
        response = self.send_command("r vbus_voltage")
        if response:
            try:
                bus_voltage = float(response)
                if self.debug:
                    print(f"dc voltage: {bus_voltage}")
                if bus_voltage < 10:
                    print("ERROR: DC Bus Undervoltage", bus_voltage)
            except ValueError:
                if self.debug:
                    print(f"Gelen yanıt sayıya çevrilemedi: {response}")
        else:
            if self.debug:
                print("ODrive'dan yanıt alınamadı.")

    def get_current_state(self):
        """current state return"""
        self.send_command("r axis0.current_state")
        if self.debug:
            print("Requesting current state.")

    def get_motor_current(self):
        """Motor akımını al."""
        self.send_command("r axis0.motor.foc.Iq_setpoint")
        return self.read_response()

    def wait_for_odrive(self):
        """ODrive'ı bulana kadar bekle."""
        self.send_command("r axis0.current_state")
        current_state = self.read_response()
        if self.debug:
            print("current state before find odrive : ", current_state)

        while current_state == 0:  # AXIS_STATE_UNDEFINED:
            if self.debug:
                print("Found ODrive")
            current_state = self.read_response()

    def set_pid(self):
        """PID ayarlarını yapılandır."""
        self.send_command("w axis0.controller.config.vel_gain 0.275")
        self.send_command(
            "w axis0.controller.config.vel_integrator_gain 1.375")
        self.send_command("w axis0.controller.config.pos_gain 25")
        self.send_command("w axis0.controller.config.vel_limit 43")
        self.send_command("w axis0.controller.config.vel_ramp_rate 2")
        self.send_command("w axis0.controller.config.inertia 0")
        self.send_command(
            "w axis0.controller.config.input_filter_bandwidth 20")
        if self.debug:
            print("PID setted!")

    def set_current(self):
        """Motor akımı ayarlarını yapılandır."""
        self.send_command("w axis0.config.motor.current_soft_max 8")
        self.send_command("w axis0.config.motor.current_hard_max 20")
        self.send_command(
            "w axis0.config.motor.current_control_bandwidth 1000")
        if self.debug:
            print("Current setting done!")

    def set_volt(self):
        """Voltaj ayarlarını yapılandır."""
        self.send_command("w config.dc_bus_undervoltage_trip_level 10.5")
        self.send_command("w config.dc_bus_overvoltage_trip_level 35")
        if self.debug:
            print("Volt setting done!")

    def set_uart(self):
        """UART ayarlarını yapılandır."""
        self.send_command("w config.enable_uart_a 1")
        self.send_command("w config.uart_a_baudrate 19200")
        self.send_command("w config.uart0_protocol 3")  # ASCII_AND_STDOUT
        self.send_command("w config.gpio6_mode 4")  # UART_A
        self.send_command("w config.gpio7_mode 4")  # UART_A
        self.send_command("w config.gpio10_mode 0")  # DIGITAL
        if self.debug:
            print("UART setting done!")

    def clear_error(self):
        response = self.send_command("r clear_errors")
        if self.debug:
            print("Error cleared! Response:", response)

    def full_calib(self):
        """Tam kalibrasyon sürecini başlat."""
        if self.debug:
            print("Before mode full calib: ")
        self.clear_error()
        current_state = self.send_command("r axis0.current_state")
        if current_state is not None and current_state != 3:
            # Kalibrasyon moduna geç
            self.send_command("w axis0.requested_state 3")
            if self.debug:
                print("Full calibration sequence started.")
        while True:
            current_state = int(self.send_command("r axis0.current_state"))
            if current_state == 1:
                if self.debug:
                    print("Full calibration sequence completed successfully.")
                break

    def set_trap_traj(self):
        """ODrive'daki hataları temizle."""
        self.send_command("w axis0.controller.config.control_mode 3 ")
        self.send_command("w axis0.controller.config.input_mode 5 ")
        self.send_command("w axis0.controller.input_vel 0 ")
        self.send_command("w axis0.controller.input_torque 0 ")
        self.send_command("w axis0.controller.config.control_mode 3 ")

        self.send_command("w axis0.trap_traj.config.vel_limit 11 ")
        self.send_command("w axis0.trap_traj.config.accel_limit 1000 ")
        self.send_command("w axis0.trap_traj.config.decel_limit 1000 ")
        if self.debug:
            print("trap traj setted ...")

    def set_vel_control(self, vel):
        # CONTROL_MODE_VELOCITY_CONTROL
        self.send_command("w axis0.controller.config.control_mode 2 ")
        # INPUT_MODE_PASSTHROUGH
        self.send_command("w axis0.controller.config.input_mode 1")
        self.send_command(f"w axis0.controller.input_vel {vel}")

        if self.debug:
            print("hız control mode  setted ...")

    def mode_control(self):
        if self.debug:
            print("mode control:")
        self.send_command("r axis0.controller.config.control_mode ")
        self.send_command("r axis0.controller.config.input_mode ")
        self.send_command("r axis0.controller.input_vel")

    def set_closed_loop(self):
        """Enable closed loop control."""
        if self.debug:
            print("Enabling closed loop control...")
        self.clear_error()
        self.send_command("w axis0.requested_state 8")

        current_state = self.send_command("r axis0.current_state ")
        if current_state != 8:
            procedure_result = self.send_command("r axis0.procedure_result")
            if self.debug:
                print("Received result:", procedure_result)
            self.error_control()
            self.send_command("w axis0.requested_state 8")
        if current_state == 8 and self.debug:
            print("Motor closed loop mode active")

    def error_control(self):
        """Hata kontrolü."""

        result_string = self.send_command("r axis0.procedure_result")
        if self.debug:
            print("Received result:", result_string)  # Gelen tüm veriyi yazdır
            print("Received result type:", type(result_string))

        # Gelen verinin None olup olmadığını ve hatayı kontrol et
        if result_string is not None:
            try:
                error_code = int(result_string)

                if error_code == 14:
                    print("NOT CALIBRATED ERROR")
                    self.full_calib()  # Kalibrasyon başlat
                elif error_code == 2:
                    print("Cancelled error")
                    self.clear_error()  # Hataları temizle
                elif error_code == 5:
                    print("Pole pair error")
                elif error_code == 4:
                    print("Encoder error")
                else:
                    print(f"Unknown error, hata kodu: {error_code}")
            except ValueError:
                print(f"Invalid response, not an integer: {result_string}")
        else:
            print("No response received from ODrive.")

        # Motor akımını oku
        current_response = self.send_command("r axis0.motor.foc.Iq_setpoint")
        if self.debug:
            print(f"Motor current response: {current_response}")

        return current_response  # Akımı döndür

    def torq(self, threshold=1):
        """return 1: tork algılandı :"""
        """Tork değerini okur ve ani değişimlerde motoru durdurur."""
        try:
            torq = self.send_command("r axis0.motor.foc.Iq_measured")
            torq_value = float(torq) if torq else None

            if torq_value is not None:
                print(f"Tork: {torq_value}")

                # Ani tork değişimini kontrol et
                if abs(torq_value) > threshold:
                    start_time = time.time()

                    print(f"Ani tork değişimi algılandı! Tork: {torq_value}")
                    return 1
                    # TODO: kaldır
                    # Motoru IDLE moduna geçir
                    self.send_command("w axis0.requested_state 1")
                    stop_time = time.time()
                    print("Motor IDLE moduna geçti.")

                    # Geçen süreyi hesapla
                    elapsed_time = stop_time - start_time
                    print(f"Motorun durmasına kadar geçen süre: {
                          elapsed_time} saniye")

            else:
                print("Tork değeri alınamadı.")

        except (ValueError, TypeError):
            print(f"Tork değeri geçersiz: {torq}")

    def torq11(self, threshold=3.0):
        """Detects torque changes and stops motor if threshold is exceeded."""
        try:
            torq = self.send_command("r axis0.motor.foc.Iq_measured")
            torq_value = float(torq) if torq else None

            if torq_value is not None:
                if self.debug:
                    print(f"Tork: {torq_value}")

                if self.previous_torque is not None:
                    torque_change = abs(torq_value - self.previous_torque)
                    if self.debug:
                        print(f"Tork değişimi: {torque_change} previous_torque: {
                              self.previous_torque}")

                    if torque_change > threshold:
                        start_time = time.time()
                        if self.debug:
                            print(f"Ani tork değişimi algılandı! Tork: {
                                  torq_value}")

                        # Motoru IDLE moduna geçir
                        self.send_command("w axis0.requested_state 1")
                        stop_time = time.time()
                        if self.debug:
                            print("Motor IDLE moduna geçti.")

                        elapsed_time = stop_time - start_time
                        if self.debug:
                            print(f"Motorun durmasına kadar geçen süre: {
                                  elapsed_time} saniye")
                        return 1  # Ani değişim algılandı, motor durduruldu
                else:
                    if self.debug:
                        print("İlk tork değeri alındı, önceki tork mevcut değil.")

                self.previous_torque = torq_value
            else:
                if self.debug:
                    print("Tork değeri alınamadı.")

        except (ValueError, TypeError) as e:
            if self.debug:
                print(f"Tork değeri geçersiz: {torq}. Hata: {e}")

    def torq2(self, threshold=1.0):
        """Tork değerini okur ve ani değişimlerde motoru durdurur."""
        try:
            # Tork verisini al
            torq = self.send_command("r axis0.motor.foc.Iq_measured")
            torq_value = float(torq) if torq else None

            if torq_value is not None:
                print(f"Tork: {torq_value}")

                # Eğer önceki tork değeri mevcutsa, değişimi kontrol et
                if self.previous_torque is not None:
                    torque_change = abs(torq_value - self.previous_torque)

                    print(f"Tork değişimi: {torque_change} previous_torque: {
                          self.previous_torque}")

                    # Ani tork değişimini kontrol et
                    if torque_change > threshold:
                        start_time = time.time()
                        print(f"Ani tork değişimi algılandı! Tork: {
                              torq_value}")

                        # Motoru IDLE moduna geçir (motoru durdur)
                        rs = self.send_command("w axis0.requested_state 1")

                        stop_time = time.time()
                        print("Motor IDLE moduna geçti.")

                        # Geçen süreyi hesapla
                        elapsed_time = stop_time - start_time
                        print(f"Motorun durmasına kadar geçen süre: {
                              elapsed_time} saniye")
                        return 1  # Ani değişim algılandı, motor durduruldu
                else:
                    print("İlk tork değeri alındı, önceki tork mevcut değil.")

                # Önceki tork değerini güncelle
                self.previous_torque = torq_value
            else:
                print("Tork değeri alınamadı.")

        except (ValueError, TypeError) as e:
            print(f"Tork değeri geçersiz: {torq}. Hata: {e}")

    # TODO: bunu kaldır
    def vel111(self, threshold=1.0):
        """Velocity control logic."""
        current_velocity = float(self.send_command("r axis0.vel_estimate"))
        if current_velocity < threshold and self.last_velocity >= threshold:
            if self.debug:
                print("Çarptı!")
            self.stop_motor()

        self.last_velocity = current_velocity

    def stop_motor(self):
        """Stops the motor."""
        self.send_command("w axis0.requested_state 1")

    def pos(self, target):
        current_pos = float(odrive.send_command("r axis0.pos_estimate"))
        if (current_pos == target):
            print("hedefe ulaşıldı: ", current_pos)

    def vel(self, threshold=5 ):
        """Hız kontrol fonksiyonu."""
        current_velocity = float(self.send_command("r axis0.vel_estimate"))
        if self.debug:
            print(f"Current velocity: {current_velocity}")
        if (current_velocity >= 1 ):
            print("ilk hız sıfırdan farklı ")
            if abs(current_velocity-self.last_velocity) <= threshold:
                print(f"hız değisimi algılandı {
                      current_velocity} self.last_velocity: {self.last_velocity}")
                return 1
                print("Çarptı****************!")
                self.stop_motor()

        self.last_velocity = current_velocity

    def get_pos_estimate(self):
        """Pozisyon tahminini al."""
        pos_estimate = self.send_command("r axis0.pos_estimate")
        if self.debug:
            print(f"Position estimate: {pos_estimate}")
        return pos_estimate

    def send_input_pos(self, x):
        """Giriş pozisyonu gönder."""
        self.send_command(f"w axis0.controller.input_pos {x}")
        if self.debug:
            print(f"Input position sent: {x}")

    def send_input_pos2(self):
        """Pozisyonu sıfıra ayarla."""
        self.send_command("w axis0.controller.input_pos 0")
        if self.debug:
            print("Input position reset to 0")

    def send_request_state(self, x):
        """Motorun durumunu değiştir."""
        self.send_command(f"w axis0.requested_state {x}")
        if self.debug:
            print(f"Requested state: {x}")

    def stop_motor(self):
        """Motoru durdur."""
        start_time = time.time()
        rs = self.send_command("w axis0.requested_state 1")
        if (rs == None):
            end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"Motorun durma komutu işlenme süresi: {
              elapsed_time:.8f} saniye  : milisaniye cinsinden :{elapsed_time * 1000} responce:  {rs}")
        if self.debug:
            print("Motor stopped (IDLE mode)")

    def get_current_state(self):
        return int(self.send_command("r axis0.current_state"))

    def is_blokked(self, tork, vel):
        if (tork == 1 and vel == 1):
            self.stop_motor()
            print(
                f"stopped!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! tork:{tork},vel:{vel}")

    def detect_torque_change(self,threshold):
        """Tork değişimini sürekli izleyerek ani değişimleri yakalar."""
        # Torku al
        torq = self.send_command("r axis0.motor.foc.Iq_measured")
        try:
            torq_value = float(torq) if torq else None
        except (ValueError, TypeError):
            print(f"Geçersiz tork değeri: {torq}")
            return

        if torq_value is not None:
            print(f"Tork: {torq_value}")

            # Önceki tork yoksa (ilk okuma) güncelle ve çık
            if self.previous_torque is None:
                self.previous_torque = torq_value
                return

            # Tork değişimini hesapla
            torque_change = abs(torq_value - self.previous_torque)

            # Eğer tork değişimi eşik değeri aşıyorsa
            if torque_change > self.threshold:
                print(f"Ani tork değişimi algılandı! Tork değişimi: {torque_change} tork:{torq_value} previous tork: {self.previous_torque}")
                # self.stop_motor()
                return 1

            # Önceki torku güncelle
            self.previous_torque = torq_value


odrive = ODriveController(debug=True)  # Debug modu başlarken açık


def main():
    try:
        # odrive.clear_error()

        # voltage contol
        odrive.get_bus_voltage()
        # find odrive
        odrive.wait_for_odrive()

        # set pıd
        odrive.set_pid()

        # set current
        odrive.set_current()

        # set uart
        odrive.set_uart()

        # set volt
        odrive.set_volt()

        # del error
        # odrive.clear_error()

        # # Trap traj ayarlarını yapılandır
        odrive.set_trap_traj()

        # # cloosed loop active
        odrive.set_closed_loop()
        odrive.set_vel_control(1)

        odrive.torq()

    except KeyboardInterrupt:
        print("Program sonlandırıldı.")

    finally:
        # odrive.close()
        print("portu kapat ")


def arm_move():
    while True:
        # TODO: eğer konumu belirlenen aralık dışındaysa 0 a getir ondan sonra loop a al yok sa çalışmıyor 0 ku-onumunda oldugundan emin ol

        # odrive.send_command("w axis0.controller.input_pos 0.5")

        if (odrive.torq() == 1):
            print("döndü 1 ")
            break
        current_pos = float(odrive.send_command("r axis0.pos_estimate"))
        # odrive.send_command("w axis0.controller.input_pos 0.5")
        # odrive.send_input_pos(0)
        # time.sleep(5)
        # while(current_pos != 0.5):
        #     print("bekler current pos:",current_pos)
        #     odrive.send_input_pos(0.5)

        #     time.sleep(0.5)

        # if current_pos > 0.4:
        #     odrive.send_input_pos(0)
        # elif current_pos < 0.01:
        #     odrive.send_input_pos(0.5)
        #     time.sleep(0.5)
        #     count += 1
        #     print("count", count)

        # if count > 10:
        #     odrive.send_request_state(1)
        #     print("bittiii")
        #     break


if __name__ == "__main__":
    main()

    odrive.set_closed_loop()
    odrive.set_vel_control(3)
    odrive.mode_control()
    count = 0

    while True:

        # odrive.vel()
        # odrive.is_blokked(odrive.torq(),odrive.vel())
        odrive.is_blokked(odrive.detect_torque_change(1),odrive.vel())
        
        # odrive.detect_torque_change(1)
        # TODO: ne kadar sürede durduğunu bul

        # if (odrive.torq() == 1):
        #     print("döndü 1 ")
        #     odrive.stop_motor()
        #     break

        if (odrive.get_current_state() == 1):
            break

        current_pos = float(odrive.send_command("r axis0.pos_estimate"))
        print("gitlens deneme ")
        print("current pos:", current_pos)
