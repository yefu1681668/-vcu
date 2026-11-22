%% VCU 整车控制器 Simulink 模型自动生成脚本
% -------------------------------------------------------------------------
% 运行本脚本可一键搭建 VCU 应用层模型骨架（含输入/状态机/扭矩控制/输出），
% 并在 Stateflow 中建立整车状态机。适合作为 MBD 开发起点。
%
% 使用：MATLAB R2021b+，需 Simulink / Stateflow 工具箱
%   >> cd model
%   >> build_model
%
% 生成：VCU_Application.slx
%
% Author: yefu1681668
% Version: 1.2

clear; clc;
modelName = 'VCU_Application';
close_system(modelName, 0);
new_system(modelName);
open_system(modelName);

%% ===== 顶层架构（4 个子系统） =====
% 1. Signal_Conditioning  信号调理
% 2. State_Machine        整车状态机（Stateflow）
% 3. Torque_Control       扭矩控制
% 4. Fault_Management     故障诊断

set_param(modelName, 'SolverType', 'Fixed-step');
set_param(modelName, 'FixedStep', '0.01');   % 10 ms 任务周期，与 DBC 周期一致

% ---- 输入端口 ----
inportNames = {'AccelPedal', 'BrakePedal', 'GearReq', 'KeyState', ...
               'DriveMode', 'SOC', 'BattVoltage', 'BusVoltage', ...
               'MotorSpeed', 'BMS_Alive', 'MCU_Alive'};
for i = 1:numel(inportNames)
    add_block('simulink/Sources/In1', [modelName '/' inportNames{i}], ...
              'Position', [40, 60*i, 70, 60*i+30]);
end

% ---- 子系统 ----
sysPos = [220, 40, 400, 180];
add_block('simulink/Ports & Subsystems/Subsystem', ...
          [modelName '/Signal_Conditioning'], 'Position', sysPos);
add_block('simulink/Ports & Subsystems/Subsystem', ...
          [modelName '/State_Machine'],        'Position', sysPos + [0, 200, 0, 200]);
add_block('simulink/Ports & Subsystems/Subsystem', ...
          [modelName '/Torque_Control'],       'Position', sysPos + [0, 400, 0, 400]);
add_block('simulink/Ports & Subsystems/Subsystem', ...
          [modelName '/Fault_Management'],     'Position', sysPos + [260, 200, 260, 200]);

% ---- 输出端口 ----
outportNames = {'VCU_State', 'TorqueReq', 'FaultLevel', 'Ready', 'HV_Enable'};
for i = 1:numel(outportNames)
    add_block('simulink/Sinks/Out1', [modelName '/' outportNames{i}], ...
              'Position', [620, 60*i, 650, 60*i+30]);
end

%% ===== Stateflow 整车状态机 =====
try
    sfnew;
    stateflowChart = [modelName '/State_Machine/VCU_Stateflow'];
    add_block('sflib/Chart', stateflowChart, 'Position', [50, 50, 180, 180]);
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.Chart', 'Name', 'VCU_Stateflow');
    if isempty(chart)
        warning('Stateflow 图表未创建，请手动在 State_Machine 子系统中添加 Chart');
    else
        states = {'INIT','PRECHARGE','READY','DRIVING','REGEN','FAULT','SHUTDOWN'};
        y = 30;
        for k = 1:numel(states)
            s = Stateflow.State(chart);
            s.Name = states{k};
            s.Position = [40, y, 100, 40];
            y = y + 60;
        end
        disp('Stateflow 状态已生成: INIT/PRECHARGE/READY/DRIVING/REGEN/FAULT/SHUTDOWN');
    end
catch ME
    warning('Stateflow 自动生成跳过: %s', ME.message);
    disp('请在 State_Machine 子系统中手动添加 Stateflow Chart，参考 model/images/state_machine.svg');
end

%% ===== 保存 =====
save_system(modelName, [pwd '/' modelName '.slx']);
disp(['已生成模型: ' pwd '/' modelName '.slx']);
disp('下一步：参照 docs/02_软件设计说明.md 补齐各子系统内部算法，');
disp('        参照 src/vcu_core.py 中的参数与规则配置 Lookup Table。');
