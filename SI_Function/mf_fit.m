function [result] = mf_fit(id_data,true_data)
%result = mu_fit(id_data, true_data)
%適合率を計算します。
%result    : 適合率
%id_data   : 同定後のデータ、予測値
%true_data : 真値、測定値

true_data_ave = mean(true_data);
result = (1 - (sqrt(sum((true_data-id_data).^2)))/(sqrt(sum((true_data-true_data_ave).^2))) )*100;
end

