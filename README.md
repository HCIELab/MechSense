# MechSenseUI
#Last Updated: 04/23/2023  
#Description: This Github link contains the User Interface for MechSense (CHI'23) as well as an example fishing game.  


---Folder Structure---  

1) MechSense UI (Folder).  
  a) MechSense_UI.pde (File, processing file that hosts the UI).    
  b) utils.pde (File, has the polynomial coefficients for equation used in MechSense_UI.pde).    
  c) assets (Folder, houses the graphics used for the fishing game).   
  
2) FDC2214 (Folder, FDC2214 Arduino compatible Capacitive Sensing library-- required for code to run)

3) MechSense_CapSense (Folder, Arduino code that sends capacitive sensor values to Serial/Processing-- required to upload to microcontroller)

---Requirements---  

1) Processing 2
2) Arduino/ microcontroller with MechSense code run on it (see "MechSense_CapSense" file)
3) FDC2214 Arduino Library installation (see "FDC2214" folder)
4) FDC2214 Sensing Board 
5) MechSense 3D printed object with stator connected to CH0, CH1, CH2 on FDC2214. 

---How to Use---

1) Upload MechSense_Capsense code to Arduino (ensuring that the FDC2214 library is also installed)
2) Run the processing file 'MechSense_UI.pde" (Note that the processing file  will not run unless it is connected to a microcontroller, otherwise you will get a grey screen) 
3) Press on the calibration button (you may change calibration specifics in the code)
4) As it's calibrating, you can toggle to see the waveforms through "draw_waveforms"
5) Once calibration is finished, a UI with direction of motion, angular position, speed (rpm) will appear
6) Waveform button, allows you to see the wavforms and segmentation (can toggle)
7) Fishing Game button, allows you to enter fishing game mode (can toggle)

---Notes---

1) Rotating at higher speeds is more likely to produce error as it changes the shape of the polynomial graph (see paper* for more details) 
2) Recommended if using laptop to have it connected to ground, as that establishes a more stable ground
3) Any Wires in the system must not be moving and hand proximity should be reasonable to avoid noise (see paper*)
4) The vertical distance between stator and rotor must be kept constant, otherwise errors might accumulate

--- Any Questions/ Notes --- 

If you have any questions or feedback, please contact malalawi@mit.edu 

--- Paper* --- 

Marwa AlAlawi, Noah Pacik-Nelson, Junyi Zhu, Ben Greenspan, Andrew Doan, Brandon Wong, Benjamin Owen-Block, Shanti Mickens, Wilhelm Schoeman, Michael Wessely, Andreea Danielescu, Stefanie Mueller.
MechSense: A Design and Fabrication Pipeline for Integrating Rotary Encoders into 3D Printed Mechanisms
https://dl.acm.org/doi/10.1145/3544548.3581361

