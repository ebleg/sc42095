function [outst] = build_PIDD(inst)
    s = tf('s');
    outst = inst;
    outst.tf = inst.Kp*(1 + (1/s)*(1/inst.Ti))...
                      *(1 + inst.Td1*s/(s*inst.Td1/inst.N + 1))...
                      *(1 + inst.Td2*s/(s*inst.Td2/inst.N + 1));
end

