import { useEffect } from 'react';
import { X, CheckCircle, AlertCircle, AlertTriangle, Info } from 'lucide-react';
import { useRobotStore } from '../../store/robotStore';

const iconMap = {
  success: CheckCircle,
  error: AlertCircle,
  warning: AlertTriangle,
  info: Info,
};

const colorMap = {
  success: 'bg-green-50 border-green-200 text-green-800',
  error: 'bg-red-50 border-red-200 text-red-800',
  warning: 'bg-yellow-50 border-yellow-200 text-yellow-800',
  info: 'bg-blue-50 border-blue-200 text-blue-800',
};

const iconColorMap = {
  success: 'text-green-500',
  error: 'text-red-500',
  warning: 'text-yellow-500',
  info: 'text-blue-500',
};

export function Notifications() {
  const { notifications, removeNotification } = useRobotStore();

  // Auto-remove notifications after 5 seconds
  useEffect(() => {
    notifications.forEach((notification) => {
      const timeout = setTimeout(() => {
        removeNotification(notification.id);
      }, 5000);

      return () => clearTimeout(timeout);
    });
  }, [notifications, removeNotification]);

  if (notifications.length === 0) return null;

  return (
    <div className="fixed bottom-4 right-4 z-50 space-y-2 w-80">
      {notifications.map((notification) => {
        const Icon = iconMap[notification.level];
        return (
          <div
            key={notification.id}
            className={`flex items-start gap-3 p-3 rounded-lg border shadow-lg ${colorMap[notification.level]}`}
          >
            <Icon className={`w-5 h-5 flex-shrink-0 ${iconColorMap[notification.level]}`} />
            <div className="flex-1 min-w-0">
              <div className="font-medium text-sm">{notification.title}</div>
              <div className="text-sm opacity-80">{notification.message}</div>
            </div>
            <button
              className="flex-shrink-0 opacity-60 hover:opacity-100"
              onClick={() => removeNotification(notification.id)}
            >
              <X className="w-4 h-4" />
            </button>
          </div>
        );
      })}
    </div>
  );
}
