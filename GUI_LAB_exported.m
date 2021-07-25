classdef GUI_LAB_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure          matlab.ui.Figure
        Slider            matlab.ui.control.Slider
        EditField         matlab.ui.control.NumericEditField
        INICIARButton     matlab.ui.control.Button
        EditField_2       matlab.ui.control.NumericEditField
        EditField_3       matlab.ui.control.NumericEditField
        Slider_2          matlab.ui.control.Slider
        Slider_3          matlab.ui.control.Slider
        Slider_4          matlab.ui.control.Slider
        Slider_5          matlab.ui.control.Slider
        Slider_6          matlab.ui.control.Slider
        EditField_4       matlab.ui.control.NumericEditField
        EditField_5       matlab.ui.control.NumericEditField
        EditField_6       matlab.ui.control.NumericEditField
        ngulosFijosLabel  matlab.ui.control.Label
        RollLabel         matlab.ui.control.Label
        PitchLabel        matlab.ui.control.Label
        YawLabel          matlab.ui.control.Label
        Label             matlab.ui.control.Label
        Label_2           matlab.ui.control.Label
        Label_3           matlab.ui.control.Label
        PosicinLabel      matlab.ui.control.Label
        XLabel            matlab.ui.control.Label
        YLabel            matlab.ui.control.Label
        ZLabel            matlab.ui.control.Label
        Label_4           matlab.ui.control.Label
        Label_5           matlab.ui.control.Label
        Label_6           matlab.ui.control.Label
        q1Label           matlab.ui.control.Label
        q2Label           matlab.ui.control.Label
        q3Label           matlab.ui.control.Label
        q4Label           matlab.ui.control.Label
        q5Label           matlab.ui.control.Label
        q6Label           matlab.ui.control.Label
        UIAxes            matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        JointPosition % Description
    end
    
    methods (Access = private)
        
        function plotRobot(app,i,q)
            L1 = 486.5/100; L2 = 150/100; L3 = 700/100; L4=600/100; L5 = 65/100;     % Dimensiones
            
            body1 = rigidBody('body1');
            jnt1 = rigidBodyJoint('jnt1','revolute');
            tform = trvec2tform([0, 0, L1]); % User defined
            setFixedTransform(jnt1,tform);
            body1.Joint = jnt1;
            robot = rigidBodyTree;
            addBody(robot,body1,'base')
            body2 = rigidBody('body2');
            jnt2 = rigidBodyJoint('jnt2','revolute');
            tform2 = trvec2tform([L2, 0, 0])*eul2tform([pi/2, -pi/2, 0]); % User defined
            setFixedTransform(jnt2,tform2);
            body2.Joint = jnt2;
            addBody(robot,body2,'body1'); % Add body2 to body1
            body3 = rigidBody('body3');
            body4 = rigidBody('body4');
            jnt3 = rigidBodyJoint('jnt3','revolute');
            jnt4 = rigidBodyJoint('jnt4','revolute');
            tform3 = trvec2tform([L3, 0, 0]); % User defined
            tform4 = trvec2tform([0, -L4, 0])*eul2tform([0, 0, pi/2]); % User defined
            setFixedTransform(jnt3,tform3);
            setFixedTransform(jnt4,tform4);
            body3.Joint = jnt3;
            body4.Joint = jnt4;
            addBody(robot,body3,'body2'); % Add body3 to body2
            addBody(robot,body4,'body3'); % Add body4 to body3
            body5 = rigidBody('body5');
            body6 = rigidBody('body6');
            jnt5 = rigidBodyJoint('jnt5','revolute');
            jnt6 = rigidBodyJoint('jnt6','revolute');
            tform5 = eul2tform([0, 0, -pi/2]); % User defined
            tform6 = eul2tform([0, 0, pi/2]); % User defined
            setFixedTransform(jnt5,tform5);
            setFixedTransform(jnt6,tform6);
            body5.Joint = jnt5;
            body6.Joint = jnt6;
            addBody(robot,body5,'body4'); % Add body5 to body4
            addBody(robot,body6,'body5'); % Add body6 to body5
            
            bodyEndEffector = rigidBody('endeffector');
            tform7 = trvec2tform([0, 0, L5])*eul2tform([pi, 0, 0]); % User defined
            setFixedTransform(bodyEndEffector.Joint,tform7);
            addBody(robot,bodyEndEffector,'body6');
            JointName = {'jnt1' 'jnt2' 'jnt3' 'jnt4' 'jnt5' 'jnt6'}; 
            app.JointPosition{i} = q;
            %JointPosition = {0 0 0 0 0 0};
            config = struct('JointName',JointName,'JointPosition',app.JointPosition);
            tform = getTransform(robot,config,'endeffector','base'); %MTH
            posicion = tform2trvec(tform); %Vector de traslacion 
            angulos = tform2eul(tform); %Euler ZYX = Angulos fijos 
            % Muestra Ángulos y Posición
            
            app.Label.Text = sprintf('%0.4f',angulos(1));
            app.Label_2.Text = sprintf('%0.4f',angulos(2));
            app.Label_3.Text = sprintf('%0.4f',angulos(3));
            
            app.Label_4.Text = sprintf('%0.4f',posicion(1)*100);
            app.Label_5.Text = sprintf('%0.4f',posicion(2)*100);
            app.Label_6.Text = sprintf('%0.4f',posicion(3)*100);
            
            
            axis(app.UIAxes,'off')
            show(robot,config);
            axis( [-5 10 -5 10 -5 15]); 
            Frame = getframe(gcf);
            [image, Map] = frame2im(Frame);
            imshow(image,"Parent",app.UIAxes)
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: INICIARButton
        function INICIARButtonPushed(app, event)
            app.UIAxes.Visible;
            app.JointPosition = {0 0 0 0 0 0};
            plotRobot(app,1,0)            
        end

        % Value changing function: Slider
        function SliderValueChanging(app, event)
            changingValue = event.Value;
            app.EditField.Value = changingValue;
            plotRobot(app,1,changingValue)
        end

        % Value changed function: EditField
        function EditFieldValueChanged(app, event)
            value = app.EditField.Value;
            app.Slider.Value = value; 
            plotRobot(app,1,value)
        end

        % Value changing function: Slider_2
        function Slider_2ValueChanging(app, event)
            changingValue = event.Value;
            app.EditField_2.Value = changingValue;
            plotRobot(app,2,changingValue)
        end

        % Value changed function: EditField_2
        function EditField_2ValueChanged(app, event)
            value = app.EditField_2.Value;
            app.Slider_2.Value = value; 
            plotRobot(app,2,value)
        end

        % Value changed function: EditField_3
        function EditField_3ValueChanged(app, event)
            value = app.EditField_3.Value;
            app.Slider_3.Value = value; 
            plotRobot(app,3,value)
        end

        % Value changing function: Slider_3
        function Slider_3ValueChanging(app, event)
            changingValue = event.Value;
            app.EditField_3.Value = changingValue;
            plotRobot(app,3,changingValue)
        end

        % Value changing function: Slider_4
        function Slider_4ValueChanging(app, event)
            changingValue = event.Value;
            app.EditField_4.Value = changingValue;
            plotRobot(app,4,changingValue)
        end

        % Value changing function: Slider_5
        function Slider_5ValueChanging(app, event)
            changingValue = event.Value;
            app.EditField_5.Value = changingValue;
            plotRobot(app,5,changingValue)            
        end

        % Value changing function: Slider_6
        function Slider_6ValueChanging(app, event)
            changingValue = event.Value;
            app.EditField_6.Value = changingValue;
            plotRobot(app,6,changingValue)
        end

        % Value changed function: EditField_4
        function EditField_4ValueChanged(app, event)
            value = app.EditField_4.Value;
            app.Slider_4.Value = value; 
            plotRobot(app,4,value)
        end

        % Value changed function: EditField_5
        function EditField_5ValueChanged(app, event)
            value = app.EditField_5.Value;
            app.Slider_5.Value = value; 
            plotRobot(app,5,value)
        end

        % Value changed function: EditField_6
        function EditField_6ValueChanged(app, event)
            value = app.EditField_6.Value;
            app.Slider_6.Value = value; 
            plotRobot(app,6,value)
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 995 686];
            app.UIFigure.Name = 'MATLAB App';

            % Create Slider
            app.Slider = uislider(app.UIFigure);
            app.Slider.Limits = [-3.14159265358979 3.14159265358979];
            app.Slider.ValueChangingFcn = createCallbackFcn(app, @SliderValueChanging, true);
            app.Slider.FontWeight = 'bold';
            app.Slider.Position = [34 587 406 3];

            % Create EditField
            app.EditField = uieditfield(app.UIFigure, 'numeric');
            app.EditField.Limits = [-3.14159265358979 3.14159265358979];
            app.EditField.ValueChangedFcn = createCallbackFcn(app, @EditFieldValueChanged, true);
            app.EditField.HorizontalAlignment = 'center';
            app.EditField.FontWeight = 'bold';
            app.EditField.Position = [463 577 44 22];

            % Create INICIARButton
            app.INICIARButton = uibutton(app.UIFigure, 'push');
            app.INICIARButton.ButtonPushedFcn = createCallbackFcn(app, @INICIARButtonPushed, true);
            app.INICIARButton.FontWeight = 'bold';
            app.INICIARButton.Position = [187 629 100 22];
            app.INICIARButton.Text = 'INICIAR';

            % Create EditField_2
            app.EditField_2 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_2.Limits = [-1.5707963267949 2.61799387799149];
            app.EditField_2.ValueChangedFcn = createCallbackFcn(app, @EditField_2ValueChanged, true);
            app.EditField_2.HorizontalAlignment = 'center';
            app.EditField_2.FontWeight = 'bold';
            app.EditField_2.Position = [463 515 44 22];

            % Create EditField_3
            app.EditField_3 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_3.Limits = [-4.1538836197465 1.37881010907552];
            app.EditField_3.ValueChangedFcn = createCallbackFcn(app, @EditField_3ValueChanged, true);
            app.EditField_3.HorizontalAlignment = 'center';
            app.EditField_3.FontWeight = 'bold';
            app.EditField_3.Position = [463 447 44 22];

            % Create Slider_2
            app.Slider_2 = uislider(app.UIFigure);
            app.Slider_2.Limits = [-1.5707963267949 2.61799387799149];
            app.Slider_2.ValueChangingFcn = createCallbackFcn(app, @Slider_2ValueChanging, true);
            app.Slider_2.FontWeight = 'bold';
            app.Slider_2.Position = [34 519 406 3];

            % Create Slider_3
            app.Slider_3 = uislider(app.UIFigure);
            app.Slider_3.Limits = [-4.1538836197465 1.37881010907552];
            app.Slider_3.ValueChangingFcn = createCallbackFcn(app, @Slider_3ValueChanging, true);
            app.Slider_3.FontWeight = 'bold';
            app.Slider_3.Position = [34 452 406 3];

            % Create Slider_4
            app.Slider_4 = uislider(app.UIFigure);
            app.Slider_4.Limits = [-2.70526034059121 2.70526034059121];
            app.Slider_4.ValueChangingFcn = createCallbackFcn(app, @Slider_4ValueChanging, true);
            app.Slider_4.FontWeight = 'bold';
            app.Slider_4.Position = [34 385 406 3];

            % Create Slider_5
            app.Slider_5 = uislider(app.UIFigure);
            app.Slider_5.Limits = [-1.5707963267949 2.35619449019234];
            app.Slider_5.ValueChangingFcn = createCallbackFcn(app, @Slider_5ValueChanging, true);
            app.Slider_5.FontWeight = 'bold';
            app.Slider_5.Position = [34 318 406 3];

            % Create Slider_6
            app.Slider_6 = uislider(app.UIFigure);
            app.Slider_6.Limits = [-3.49065850398866 3.49065850398866];
            app.Slider_6.ValueChangingFcn = createCallbackFcn(app, @Slider_6ValueChanging, true);
            app.Slider_6.FontWeight = 'bold';
            app.Slider_6.Position = [34 251 406 3];

            % Create EditField_4
            app.EditField_4 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_4.Limits = [-2.70526034059121 2.70526034059121];
            app.EditField_4.ValueChangedFcn = createCallbackFcn(app, @EditField_4ValueChanged, true);
            app.EditField_4.HorizontalAlignment = 'center';
            app.EditField_4.FontWeight = 'bold';
            app.EditField_4.Position = [463 375 44 22];

            % Create EditField_5
            app.EditField_5 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_5.Limits = [-1.5707963267949 2.35619449019234];
            app.EditField_5.ValueChangedFcn = createCallbackFcn(app, @EditField_5ValueChanged, true);
            app.EditField_5.HorizontalAlignment = 'center';
            app.EditField_5.FontWeight = 'bold';
            app.EditField_5.Position = [463 313 44 22];

            % Create EditField_6
            app.EditField_6 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_6.Limits = [-3.49065850398866 3.49065850398866];
            app.EditField_6.ValueChangedFcn = createCallbackFcn(app, @EditField_6ValueChanged, true);
            app.EditField_6.HorizontalAlignment = 'center';
            app.EditField_6.FontWeight = 'bold';
            app.EditField_6.Position = [463 241 44 22];

            % Create ngulosFijosLabel
            app.ngulosFijosLabel = uilabel(app.UIFigure);
            app.ngulosFijosLabel.FontSize = 15;
            app.ngulosFijosLabel.FontWeight = 'bold';
            app.ngulosFijosLabel.Position = [27 155 110 22];
            app.ngulosFijosLabel.Text = 'Ángulos Fijos:';

            % Create RollLabel
            app.RollLabel = uilabel(app.UIFigure);
            app.RollLabel.FontSize = 13;
            app.RollLabel.FontWeight = 'bold';
            app.RollLabel.Position = [36 122 35 22];
            app.RollLabel.Text = 'Roll:';

            % Create PitchLabel
            app.PitchLabel = uilabel(app.UIFigure);
            app.PitchLabel.FontSize = 13;
            app.PitchLabel.FontWeight = 'bold';
            app.PitchLabel.Position = [33 86 42 22];
            app.PitchLabel.Text = 'Pitch:';

            % Create YawLabel
            app.YawLabel = uilabel(app.UIFigure);
            app.YawLabel.FontSize = 13;
            app.YawLabel.FontWeight = 'bold';
            app.YawLabel.Position = [35 51 35 22];
            app.YawLabel.Text = 'Yaw:';

            % Create Label
            app.Label = uilabel(app.UIFigure);
            app.Label.FontSize = 13;
            app.Label.Position = [94 122 61 22];
            app.Label.Text = '-';

            % Create Label_2
            app.Label_2 = uilabel(app.UIFigure);
            app.Label_2.FontSize = 13;
            app.Label_2.Position = [94 86 61 22];
            app.Label_2.Text = '-';

            % Create Label_3
            app.Label_3 = uilabel(app.UIFigure);
            app.Label_3.FontSize = 13;
            app.Label_3.Position = [94 51 61 22];
            app.Label_3.Text = '-';

            % Create PosicinLabel
            app.PosicinLabel = uilabel(app.UIFigure);
            app.PosicinLabel.FontSize = 15;
            app.PosicinLabel.FontWeight = 'bold';
            app.PosicinLabel.Position = [247 155 73 22];
            app.PosicinLabel.Text = 'Posición:';

            % Create XLabel
            app.XLabel = uilabel(app.UIFigure);
            app.XLabel.FontSize = 13;
            app.XLabel.FontWeight = 'bold';
            app.XLabel.Position = [247 122 25 22];
            app.XLabel.Text = 'X:';

            % Create YLabel
            app.YLabel = uilabel(app.UIFigure);
            app.YLabel.FontSize = 13;
            app.YLabel.FontWeight = 'bold';
            app.YLabel.Position = [247 86 25 22];
            app.YLabel.Text = 'Y:';

            % Create ZLabel
            app.ZLabel = uilabel(app.UIFigure);
            app.ZLabel.FontSize = 13;
            app.ZLabel.FontWeight = 'bold';
            app.ZLabel.Position = [247 51 25 22];
            app.ZLabel.Text = 'Z:';

            % Create Label_4
            app.Label_4 = uilabel(app.UIFigure);
            app.Label_4.FontSize = 13;
            app.Label_4.Position = [279 122 61 22];
            app.Label_4.Text = '-';

            % Create Label_5
            app.Label_5 = uilabel(app.UIFigure);
            app.Label_5.FontSize = 13;
            app.Label_5.Position = [279 86 61 22];
            app.Label_5.Text = '-';

            % Create Label_6
            app.Label_6 = uilabel(app.UIFigure);
            app.Label_6.FontSize = 13;
            app.Label_6.Position = [279 51 61 22];
            app.Label_6.Text = '-';

            % Create q1Label
            app.q1Label = uilabel(app.UIFigure);
            app.q1Label.FontWeight = 'bold';
            app.q1Label.Position = [510 577 33 22];
            app.q1Label.Text = 'q1';

            % Create q2Label
            app.q2Label = uilabel(app.UIFigure);
            app.q2Label.FontWeight = 'bold';
            app.q2Label.Position = [510 515 33 22];
            app.q2Label.Text = 'q2';

            % Create q3Label
            app.q3Label = uilabel(app.UIFigure);
            app.q3Label.FontWeight = 'bold';
            app.q3Label.Position = [510 447 33 22];
            app.q3Label.Text = 'q3';

            % Create q4Label
            app.q4Label = uilabel(app.UIFigure);
            app.q4Label.FontWeight = 'bold';
            app.q4Label.Position = [510 375 33 22];
            app.q4Label.Text = 'q4';

            % Create q5Label
            app.q5Label = uilabel(app.UIFigure);
            app.q5Label.FontWeight = 'bold';
            app.q5Label.Position = [510 313 33 22];
            app.q5Label.Text = 'q5';

            % Create q6Label
            app.q6Label = uilabel(app.UIFigure);
            app.q6Label.FontWeight = 'bold';
            app.q6Label.Position = [510 241 33 22];
            app.q6Label.Text = 'q6';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.Visible = 'off';
            app.UIAxes.Position = [506 70 486 617];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = GUI_LAB_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end