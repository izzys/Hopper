% %bla
% close all
% X_PMeps =       [1e-1           , 1e-2         , 1e-3           , 1e-4            , 1e-5          , 1e-6          , 1e-7   ];
% Y_BigEig =      [-0.6684669698  , 0.3237234935 , -0.1673082087  , -0.24275639632  , -0.247824269  , -0.2483189472 , -0.24835165241  ];
% Y_LittleEig =   [0.1044881404   , 0.3237234935 , -0.0867603787  , -0.00618857139  , -0.000610325  , -0.0000810874 , -0.00020965689];
% 
% figure
% semilogx(X_PMeps,Y_BigEig,'bo','MarkerSize',7,'LineWidth',2)
% xlabel 'PMeps'
% ylabel 'Eig #1'
% figure
% semilogx(X_PMeps,Y_LittleEig,'mx','MarkerSize',7,'LineWidth',2)
% xlabel 'PMeps'
% ylabel 'Eig #2'

%bla
close all
% X_PMeps =       [ 1e-4            ,  5e-5          , 1e-5        , 5e-6          , 1e-6          ,  5e-7           , 1e-7    ];
% Y_BigEig =      [ -0.24275639632  , -0.24559598879 ,-0.247824269 ,-0.24810005686 , -0.2483189472 , -0.24834459233  ,  -0.24835165241  ];
% Y_LittleEig =   [ -0.00618857139  , -0.00306468839 ,-0.000610325 ,-0.00030793508 , -0.0000810874 , -0.00007114986  ,  -0.00020965689];
X_PMeps =       [ 2.5e-5          ,   1e-5       , 5e-6          ,   2.5e-6         , 1e-6          ,  5e-7           , 2.5e-7        , 1e-7    ];
Y_BigEig =      [ -0.246993062835 , -0.247824269 ,-0.24810005686 ,  -0.248344592333 ,-0.2483189472  , -0.24834459233  , -0.2483445923 , -0.24835165241  ];
Y_LittleEig =   [ -0.0015257562   ,-0.000610325  ,-0.00030793508 ,  -0.000071149865 , -0.0000810874 , -0.00007114986  ,-0.00007114986 ,-0.00020965689];
figure
semilogx(X_PMeps,Y_BigEig,'bo','MarkerSize',7,'LineWidth',2)
xlabel 'PMeps'
ylabel 'Eig #1'
figure
semilogx(X_PMeps,Y_LittleEig,'mx','MarkerSize',7,'LineWidth',2)
xlabel 'PMeps'
ylabel 'Eig #2'

BigEiG_error = Y_BigEig - ones(1,length(Y_BigEig))* (-0.248375853254210);
figure
semilogx(X_PMeps,abs(BigEiG_error),'bo','MarkerSize',7,'LineWidth',2)
hold on
semilogx(X_PMeps,abs(Y_LittleEig),'xm','MarkerSize',7,'LineWidth',2)
legend('#1 Eig error','#2 Eig error')
xlabel 'PMeps'