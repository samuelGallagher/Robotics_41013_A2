workspace = [-3 3 -2 2 0 2];

L1 = Link('d',0.33,'a',0.04,'alpha',-pi/2,'qlim',[deg2rad(-170),deg2rad(170)], 'offset',0); % was 'offset',pi/2
           L2 = Link('d',0,'a',0.385,'alpha',0,'qlim', [deg2rad(-65),deg2rad(145)], 'offset', 0);
           L3 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',[deg2rad(-70),deg2rad(190)],'offset', 0); % was 'offset',pi/2
           L4 = Link('d',0.34,'a',0,'alpha',pi/2,'qlim',[deg2rad(-190),deg2rad(190)], 'offset',0);
           L5 = Link('d',0,'a',0.08,'alpha',-pi/2,'qlim',[deg2rad(-135),deg2rad(135)], 'offset', 0);
           L6 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[deg2rad(-360),deg2rad(360)], 'offset', 0);

    self.model = SerialLink([L1 L2 L3 L4 L5 L6]);
    q = [0,0,0,0,0,0];
    self.model.teach(q)