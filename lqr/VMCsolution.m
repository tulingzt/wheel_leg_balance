VMCForwardSolution[Q1_, Q4_, L_] := 
    (Module[{Xb, Yb, Xc, Yc, Xd, Yd, Lbd, L0, A, B, F, q0, q1, q2, q3, q4, 
       LegGraph}, q1 = Q1*(Pi/180); q4 = Q4*(Pi/180); Xb = L[[1]]*Cos[q1]; 
       Yb = L[[1]]*Sin[q1]; Xd = L[[4]]*Cos[q4] + L[[5]]; 
       Yd = L[[4]]*Sin[q4]; Lbd = Sqrt[(Xd - Xb)^2 + (Yd - Yb)^2]; 
       A = 2*L[[2]]*(Xd - Xb); B = 2*L[[2]]*(Yd - Yb); 
       F = L[[2]]^2 + Lbd^2 - L[[3]]^2; 
       q2 = 2*ArcTan[(B + Sqrt[A^2 + B^2 - F^2])/(A + F)]; 
       Xc = Xb + L[[2]]*Cos[q2]; Yc = Yb + L[[2]]*Sin[q2]; 
       q3 = ArcTan[Xc - Xd, Yc - Yd]; 
       L0 = Sqrt[(Xc - L[[5]]/2)^2 + (Yc + 0)^2]; 
       q0 = ArcTan[Xc - L[[5]]/2, Yc + 0]; LegGraph = 
        Graphics[{LightGray, Disk[{0, 0}, 0.048], LightGray, 
          Disk[{L[[5]], 0}, 0.048], LightGray, Disk[{Xc, Yc}, 0.021], 
          PointSize[0.024], Red, Point[{{0, 0}, {L[[5]], 0}, {Xc, Yc}}], 
          Thickness[0.004], Black, Line[{{0, 0}, {Xb, Yb}, {Xc, Yc}, 
            {Xd, Yd}, {L[[5]], 0}}], Thickness[0.004], Red, 
          Line[{{L[[5]]/2, 0}, {Xc, Yc}}], Thickness[0.001], Blue, Dashed, 
          Circle[{0, 0}, L[[1]]], Thickness[0.001], Blue, Dashed, 
          Circle[{L[[5]], 0}, L[[4]]], Thickness[0.001], Red, Dashed, 
          Circle[{Xc, Yc}, L[[2]]], Null}, Frame -> True, 
         GridLines -> Automatic, PlotRange -> {{-0.4, 0.5}, {-0.2, 0.5}}]; 
       Return[{{LegGraph}, {L0}, {q0, q1, q2, q3, q4}, {q0/Degree, q1/Degree, 
          q2/Degree, q3/Degree, q4/Degree}}]; ]; )
 
VMCInverseSolution[L0_, Q0_, L_] := 
    (Module[{Xb, Yb, Xc, Yc, Xd, Yd, A, B, F, q0, q1, q2, q3, q4, q01, q12, 
       q34, q40, LegGraph}, q0 = Q0*(Pi/180); Xc = L[[5]]/2 + L0*Cos[q0]; 
       Yc = L0*Sin[q0]; q12 = ArcCos[(Xc^2 + Yc^2 - L[[1]]^2 - L[[2]]^2)/
          (2*L[[1]]*L[[2]])]; q34 = ArcCos[((Xc - L[[5]])^2 + Yc^2 - 
           L[[3]]^2 - L[[4]]^2)/(2*L[[3]]*L[[4]])]; A = Xc; 
       B = L[[2]]*Sin[q12]; F = L[[1]] + L[[2]]*Cos[q12]; 
       q01 = ArcCos[F/Sqrt[B^2 + F^2]] + ArcCos[A/Sqrt[B^2 + F^2]]; 
       A = L[[5]] - Xc; B = L[[3]]*Sin[q34]; F = L[[4]] + L[[3]]*Cos[q34]; 
       q40 = ArcCos[F/Sqrt[B^2 + F^2]] + ArcCos[A/Sqrt[B^2 + F^2]]; q1 = q01; 
       q2 = q01 - q12; q3 = q34 - q40 + Pi; q4 = Pi - q40; 
       Xb = L[[1]]*Cos[q1]; Yb = L[[1]]*Sin[q1]; 
       Xd = L[[5]] + L[[4]]*Cos[q4]; Yd = L[[4]]*Sin[q4]; 
       LegGraph = Graphics[{LightGray, Disk[{0, 0}, 0.048], LightGray, 
          Disk[{L[[5]], 0}, 0.048], LightGray, Disk[{Xc, Yc}, 0.021], 
          PointSize[0.024], Red, Point[{{0, 0}, {L[[5]], 0}, {Xc, Yc}}], 
          Thickness[0.004], Black, Line[{{0, 0}, {Xb, Yb}, {Xc, Yc}, 
            {Xd, Yd}, {L[[5]], 0}}], Thickness[0.004], Red, 
          Line[{{L[[5]]/2, 0}, {Xc, Yc}}], Thickness[0.001], Blue, Dashed, 
          Circle[{0, 0}, L[[1]]], Thickness[0.001], Blue, Dashed, 
          Circle[{L[[5]], 0}, L[[4]]], Thickness[0.001], Red, Dashed, 
          Circle[{Xc, Yc}, L[[2]]], Null}, Frame -> True, 
         GridLines -> Automatic, PlotRange -> {{-0.4, 0.5}, {-0.2, 0.5}}]; 
       Return[{{LegGraph}, {L0}, {q0, q1, q2, q3, q4}, {q0/Degree, q1/Degree, 
          q2/Degree, q3/Degree, q4/Degree}}]; ]; )
