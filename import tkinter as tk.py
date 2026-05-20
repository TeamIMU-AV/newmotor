import tkinter as tk
from tkinter import ttk

class RobotaksiSimulasyon:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotaksi Alt Seviye Motor & Fren Kontrolü")
        self.root.geometry("550x500")
        self.root.configure(bg="#f0f4f8")
        
        # --- STATE MACHINE DEĞİŞKENLERİ ---
        self.state = "STOP"          # State Machine Ana Durumları: STOP, DRIVING, DECELERATION
        self.sub_state = "NONE"      # Deceleration Alt Durumları: COASTING veya BRAKING
        self.anlik_hiz = 0.0
        self.hedef_hiz = 0.0
        
        # PI Hız Denetleyici Değişkenleri
        self.integral_hata = 0.0
        self.son_hata = 0.0
        self.Kp = 2.5                
        self.Ki = 0.5                
        self.dt = 0.1                

        # --- KULLANICI ARAYÜZÜ (UI) ---
        tk.Label(root, text="ROBOTAKSİ OTONOM KONTROL PANELİ", font=("Arial", 13, "bold"), bg="#f0f4f8", fg="#1a365d").pack(pady=15)

        frame_slider = tk.Frame(root, bg="#f0f4f8")
        frame_slider.pack(pady=10)
        tk.Label(frame_slider, text="Hedef Hız Ayarla (km/h):", font=("Arial", 10, "bold"), bg="#f0f4f8").pack()
        self.slider = tk.Scale(frame_slider, from_=0, to=50, orient=tk.HORIZONTAL, length=350, bg="#ffffff", bd=1, command=self.hedef_guncelle)
        self.slider.pack(pady=5)

        self.frame = tk.LabelFrame(root, text=" State Machine ", font=("Arial", 10, "bold"), bg="#ffffff", fg="#2d3748", padx=20, pady=20)
        self.frame.pack(pady=15, fill="both", expand="yes", padx=30)

        self.lbl_state = tk.Label(self.frame, text="State: STOP", font=("Arial", 12, "bold"), bg="#ffffff", fg="#e53e3e")
        self.lbl_state.grid(row=0, column=0, sticky="w", pady=8)

        self.lbl_anlik = tk.Label(self.frame, text="Anlık Hız: 0.00 km/h", font=("Arial", 11, "bold"), bg="#ffffff", fg="#2d3748")
        self.lbl_anlik.grid(row=1, column=0, sticky="w", pady=6)

        self.lbl_pwm = tk.Label(self.frame, text="Motor Sürücü PWM Çıkışı: 0", font=("Arial", 11), bg="#ffffff", fg="#4a5568")
        self.lbl_pwm.grid(row=2, column=0, sticky="w", pady=6)

        self.lbl_fren = tk.Label(self.frame, text="Elektronik Fren: AKTİF (%100)", font=("Arial", 11, "bold"), bg="#ffffff", fg="#e53e3e")
        self.lbl_fren.grid(row=3, column=0, sticky="w", pady=6)

        self.guncelle_dongusu()

    def hedef_guncelle(self, val):
        self.hedef_hiz = float(val)

    def guncelle_dongusu(self):
        # 1. HATA HESAPLAMA
        hata = self.hedef_hiz - self.anlik_hiz
        
        # 2. STATE MACHINE GEÇİŞLERİ
        if self.hedef_hiz == 0:
            self.state = "STOP"
            self.sub_state = "NONE"
        elif hata >= -0.5:
            self.state = "DRIVING"
            self.sub_state = "NONE"
        else:
            self.state = "DECELERATION"
            if abs(hata) > 8.0:
                self.sub_state = "BRAKING"
            else:
                self.sub_state = "COASTING"

        # 3. DURUMA GÖRE KONTROL EYLEMLERİ
        pwm_cikis = 0
        fren_yuzdesi = 0

        if self.state == "STOP":
            self.anlik_hiz = max(0.0, self.anlik_hiz - 2.5)  
            fren_yuzdesi = 100
            pwm_cikis = 0
            self.integral_hata = 0  
            
        elif self.state == "DRIVING":
            fren_yuzdesi = 0
            
            # PI Algoritması
            self.integral_hata += hata * self.dt
            pi_cikis = (self.Kp * hata) + (self.Ki * self.integral_hata)
            
            pwm_cikis = int(max(0, min(255, pi_cikis * 5))) 
            
            self.anlik_hiz += (pwm_cikis / 255.0) * 1.8 - 0.15  
            self.anlik_hiz = max(0.0, min(self.anlik_hiz, 50.0))

        elif self.state == "DECELERATION":
            pwm_cikis = 0
            self.integral_hata = 0  
            
            if self.sub_state == "BRAKING":
                fren_yuzdesi = int(min(100, abs(hata) * 6))
                self.anlik_hiz = max(0.0, self.anlik_hiz - 1.8)  
            elif self.sub_state == "COASTING":
                fren_yuzdesi = 0
                self.anlik_hiz = max(0.0, self.anlik_hiz - 0.4)  

        self.son_hata = hata

        self.arayuz_guncelle(pwm_cikis, fren_yuzdesi)
        self.root.after(100, self.guncelle_dongusu)

    def arayuz_guncelle(self, pwm, fren):
        self.lbl_anlik.config(text=f"Anlık Hız: {self.anlik_hiz:.2f} km/h")
        self.lbl_pwm.config(text=f"Motor Sürücü PWM Çıkışı: {pwm}")
        
        if self.state == "STOP":
            self.lbl_state.config(text="State: STOP", fg="#e53e3e")
            self.lbl_fren.config(text=f"Elektronik Fren: AKTİF (%{fren})", fg="#e53e3e")
        elif self.state == "DRIVING":
            self.lbl_state.config(text="State: DRIVING", fg="#38a169")
            self.lbl_fren.config(text="Elektronik Fren: SERBEST (%0)", fg="#38a169")
        elif self.state == "DECELERATION":
            if self.sub_state == "BRAKING":
                self.lbl_state.config(text="State: DECELERATION (BRAKING)", fg="#dd6b20")
                self.lbl_fren.config(text=f"Elektronik Fren: AKTİF (%{fren})", fg="#dd6b20")
            elif self.sub_state == "COASTING":
                self.lbl_state.config(text="State: DECELERATION (COASTING)", fg="#3182ce")
                self.lbl_fren.config(text="Elektronik Fren: SERBEST (%0) - Süzülüyor", fg="#3182ce")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotaksiSimulasyon(root)
    root.mainloop()