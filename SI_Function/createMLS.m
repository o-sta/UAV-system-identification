function [continuous_signal, discrete_signal, ts] = createMLS(number, seeds, min_value, max_value, length_of_time, continuous_signal_ts)
%==========================================================================
% ◎ createMLS()
% 二値信号を生成します。
% number段シフトレジスタを用いて、mix及びmaxのみを取るランダムな
% 二値信号を生成します。length_of_timeで信号の長さを大まかに設定しますが、
% サンプリング時間との兼ね合いにより多少変化することがあります。
% 結果は連続時間信号のcontinuous_signalと離散時間信号のdiscrete_signal、
% サンプリング周期(時間)のtsを返します。
%==========================================================================
% continuous_signal     二値連続時間信号
% discrete_signal       二値離散時間信号
% ts                    二値離散時間信号のサンプリング周期
% number                number段シフトレジスタ（デフォルトは5）
% seeds                 シード値 seeds mod 2^number を2進数で表したものがレジスタの状態xの初期値になるが、0になったときはランダムにシードが選択される。
% min                   二値信号の最小値
% max                   二値信号の最大値
% length_of_time        信号全体の時間
% continuous_signal_ts  連続時間信号に格納するデータのサンプリング時間

% シフトレジスタの段数からフィードバックをどのレジスタの内容をフィードバックするか決める
switch number
case 2
  feedback_register = 1;
case 3
  feedback_register = 1;
case 4
  feedback_register = 1;
case 5
  feedback_register = 2;
case 6
  feedback_register = 1;
case 7
  feedback_register = 1;
case 9
  feedback_register = 4;
case 10
  feedback_register = 3;
case 11
  feedback_register = 2;
case 15
  feedback_register = 1;
otherwise
  number = 5;
  feedback_register = 2;
end

% レジスタの初期値を設定
if mod(seeds, 2^number) == 0
  while true
    x = randi([0 1], number, 1)  % レジスタの状態xの初期値をランダムに決定
    if ~sum(x)==0                 % レジスタの値が全て0であればやり直し
      break;
    end
  end
else
  seeds_bin = seeds;
  x = zeros(number,1)
  for i = 1:number
    x(i) = mod(seeds_bin,2);
    seeds_bin = idivide(int32(seeds_bin),2);
  end
end

% 二値信号u[0 or 1]の作成　信号の数は 2^number-1 個
discrete_signal = zeros(2^number-1, 1);   % 離散時間信号discrete_signalの定義
for i = 1:2^number-1
    x = [xor(x(feedback_register),x(end));x(1:end-1)]; % フィードバック
    discrete_signal(i) = x(1);
end
% 2値信号の値を[0,1]から他の値に変更
for i = 1:2^number - 1
    if discrete_signal(i) == 1
        discrete_signal(i) = max_value;
    else
        discrete_signal(i) = min_value;
    end
end
% 連続時間信号に変換
number_of_signal = round((length_of_time/continuous_signal_ts)/numel(discrete_signal)); % 離散時間信号1個に対応する連続時間信号のデータ数
continuous_signal_times(:,1) = 0:continuous_signal_ts:continuous_signal_ts*number_of_signal*numel(discrete_signal)-continuous_signal_ts;  % 連続時間信号のタイムスタンプ作成
continuous_signal_data = zeros(size(continuous_signal_times));
for i = 1:numel(discrete_signal)  % 連続時間信号のデータを作成
    for j = (i-1)*number_of_signal+1:i*number_of_signal
        continuous_signal_data(j) = discrete_signal(i);
    end
end
continuous_signal = timeseries(continuous_signal_data, continuous_signal_times, 'name', 'Position');  % 時系列作成
ts = number_of_signal * continuous_signal_ts; % 周期の算出
% pst = xcorr(continuous_signal_data,continuous_signal_data)
% plot(pst)

end

