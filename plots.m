figure(4)
plot([0:h:total_time_Steps*h],Wback_final,'k',Linewidth=3)
hold on
plot([0:h:total_time_Steps*h],Wfront_final,'b',Linewidth=3)
xlabel('time')
ylabel('Angular speed')
legend('back wheel angular velocity','front wheel angular velocity')