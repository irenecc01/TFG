MODULE Module1
    ! Variables y configuraci�n inicial
    ! Identifier for the EGM correction
    LOCAL VAR egmident egm_id;
    ! The work object. Base Frame
    LOCAL PERS wobjdata egm_wobj := [FALSE, TRUE, "", [[0.0, 0.0, 0.0],[1.0, 0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0],[1.0, 0.0, 0.0, 0.0]]];
    ! Limits for convergence
    ! Orientation: +-0.1 degrees
    LOCAL CONST egm_minmax egm_condition_orient := [-0.1, 0.1];
    
    ! EGM pose frames.
    !LOCAL CONST pose egm_correction_frame := [[0, 0, 0], [1, 0, 0, 0]];
    !LOCAL CONST pose egm_sensor_frame     := [[0, 0, 0], [1, 0, 0, 0]];
    
    ! Description:                                         !
    ! Externally Guided motion (EGM): Control - Main Cycle !
    ! Procedimiento principal 
    PROC Main()
        ! Move to the starting position
        MoveAbsJ [[0,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]], v100, fine, tool0\WObj:=wobj0;
       
        ! EGM Joint Orientation Control
        ! Iniciar la comunicaci�n EGM para la trayectoria
        EGM_JOINT_CONTROL; !Llama al procedimiento para el control de las articulaciones usando EGM
    ENDPROC
    ! Control EGM
    PROC EGM_JOINT_CONTROL()
        ! Description:                                   !
        ! Externally Guided motion (EGM) - Joint Control !
          
        ! Release the EGM id.
        EGMReset egm_id;
        
        ! Register an EGM id.
        EGMGetId egm_id;
            
        ! Setup the EGM communication.
        !EGMSetupUC ROB_1, egm_id, "default", "ROB_1", \Pose; !Modo Pose
        EGMSetupUC ROB_1, egm_id, "default", "ROB_1", \Joint; !Modo Joint
        
        WHILE TRUE DO
            
            ! Prepare for an EGM communication session.
            EGMActJoint egm_id
                        \J1:=egm_condition_orient
                        \J2:=egm_condition_orient
                        \J3:=egm_condition_orient
                        \J4:=egm_condition_orient
                        \J5:=egm_condition_orient
                        \J6:=egm_condition_orient
                        \MaxSpeedDeviation:=20.0;
                        
            ! Start the EGM communication session
            EGMRunJoint egm_id, EGM_STOP_RAMP_DOWN, \J1 \J2 \J3 \J4 \J5 \J6 \CondTime:=5 \RampOutTime:=5;
            
            ! Release the EGM id
            EGMReset egm_id;
            WaitTime 5;
        ENDWHILE
        
        ERROR
        IF ERRNO = ERR_UDPUC_COMM THEN
            TPWrite "Communication timeout: EGM";
            TRYNEXT;
        ENDIF
    ENDPROC
ENDMODULE