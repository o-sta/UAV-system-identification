function [u,outputArg2] = createMLS(n, mm)
%==========================================================================
% ◎ createMLS()
%==========================================================================
% u = createMLS(n, mm)
% n : n段シフトレジスタ
% [min max] : 最小値 最大値 ※出力値を変更する場合
% 
while 1
    x = randi([0 1], 5, 1); % 初期値作成
    if ~all(x(:) == 0)      % 全てゼロのときはもう一回
        break;
    end
end




% %--------------------------------------------------------------------------
% outputArg1 = inputArg1
% if exist('inputArg2') ~= 0
%     outputArg2 = inputArg2
% end
% 
% mP = 5;     % M系列のmP段シフトレジスタ
% mk = 2;     % 最終段以外にフィードバックさせる信号
% 
% x = randi([0 1], mP, 1);    % 状態の初期値をランダムに決定
% x = [0 1 0 0 1]';           % 状態の初期値を指定（全て0は禁止）
% u = zeros(2^mP-1, 1);       % 出力変数の定義
% for i = 1:2^mP-1            % 信号の生成
%     x = [xor(x(mk),x(end));x(1:end-1)]; % フィードバック
%     u(i) = x(1);
% end
% for i = 1:2^mP - 1          % 2値信号の値を0,1から他の値に変更
%     if u(i) == 1
%         u(i)=1;
%     else
%         u(i)=-1;
%     end
% end
% s_t = 5;        % 信号全体の時間[s]
% s_tp = 0.001;   % 離散信号1個あたりの継続時間[s] / サンプリング時間
% s_p = round(s_t/s_tp/numel(u));               % 1個あたりの連続時間信号数
% t_data(:,1) = 0:s_tp:s_tp*s_p*numel(u)-s_tp;  % 時間信号を作成
% u_data = zeros(size(t_data));
% for i = 1:numel(u)  % 信号を作成
%     for j = (i-1)*s_p+1:i*s_p
%         u_data(j) = u(i);
%     end
% end

end

