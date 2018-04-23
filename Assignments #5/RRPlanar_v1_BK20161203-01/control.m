function u = control(t,model,x,controlLaw)

    switch controlLaw
        case 'Passive'
            u = controlLaw_Passive(t,model,x);
            
        case 'PD'
            u = controlLaw_PD(t,model,x);
            
        case 'ComputedTorque'
            u = controlLaw_ComputedTorque(t,model,x);

        case 'JacobianTranspose'
            u = controlLaw_JacobianTranspose(t,model,x);
        
        case 'ComplianceControl'
            u = controlLaw_ComplianceControl(t,model,x);

        otherwise
            u = [0; 0];
    end
    
end