/**
 * Visualization Utilities
 * Helper functions for colors, status mapping, and data visualization
 */

import { RobotState } from 'api-client';

/**
 * Status color mapping for robots
 */
export const getRobotStatusColor = (status?: string): string => {
  if (!status) return 'var(--color-neutral-500)';

  const statusLower = status.toLowerCase();

  if (statusLower.includes('charging')) {
    return 'var(--color-warning-500)';
  }
  if (statusLower.includes('idle') || statusLower.includes('waiting')) {
    return 'var(--color-neutral-500)';
  }
  if (statusLower.includes('moving') || statusLower.includes('working')) {
    return 'var(--color-success-500)';
  }
  if (statusLower.includes('error') || statusLower.includes('emergency')) {
    return 'var(--color-error-500)';
  }

  return 'var(--color-primary-500)';
};

/**
 * Get battery level color based on percentage
 */
export const getBatteryColor = (batteryLevel: number): string => {
  if (batteryLevel > 60) return 'var(--color-success-500)';
  if (batteryLevel > 30) return 'var(--color-warning-500)';
  return 'var(--color-error-500)';
};

/**
 * Get door status color
 */
export const getDoorStatusColor = (doorMode: number): string => {
  // Door modes: 0 = closed, 1 = moving, 2 = open
  switch (doorMode) {
    case 0:
      return 'var(--color-error-500)';
    case 1:
      return 'var(--color-warning-500)';
    case 2:
      return 'var(--color-success-500)';
    default:
      return 'var(--color-neutral-500)';
  }
};

/**
 * Get lift status color
 */
export const getLiftStatusColor = (liftMode: number): string => {
  // Similar to door modes
  switch (liftMode) {
    case 0:
      return 'var(--color-error-500)';
    case 1:
      return 'var(--color-warning-500)';
    case 2:
      return 'var(--color-success-500)';
    default:
      return 'var(--color-neutral-500)';
  }
};

/**
 * Interpolate between two colors
 */
export const interpolateColor = (
  color1: string,
  color2: string,
  factor: number
): string => {
  // Simple RGB interpolation
  const c1 = hexToRgb(color1);
  const c2 = hexToRgb(color2);

  if (!c1 || !c2) return color1;

  const r = Math.round(c1.r + factor * (c2.r - c1.r));
  const g = Math.round(c1.g + factor * (c2.g - c1.g));
  const b = Math.round(c1.b + factor * (c2.b - c1.b));

  return `rgb(${r}, ${g}, ${b})`;
};

/**
 * Convert hex color to RGB
 */
const hexToRgb = (hex: string): { r: number; g: number; b: number } | null => {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result
    ? {
      r: parseInt(result[1], 16),
      g: parseInt(result[2], 16),
      b: parseInt(result[3], 16),
    }
    : null;
};

/**
 * Get task status color
 */
export const getTaskStatusColor = (status: string): string => {
  const statusLower = status.toLowerCase();

  if (statusLower.includes('completed')) {
    return 'var(--color-success-500)';
  }
  if (statusLower.includes('failed') || statusLower.includes('canceled')) {
    return 'var(--color-error-500)';
  }
  if (statusLower.includes('pending') || statusLower.includes('queued')) {
    return 'var(--color-neutral-500)';
  }
  if (statusLower.includes('active') || statusLower.includes('executing')) {
    return 'var(--color-primary-500)';
  }

  return 'var(--color-neutral-500)';
};

/**
 * Format tooltip text for robots
 */
export const formatRobotTooltip = (
  fleetName: string,
  robotName: string,
  location?: { x: number; y: number; yaw: number },
  batteryLevel?: number
): string => {
  let tooltip = `${fleetName}/${robotName}`;

  if (location) {
    tooltip += `\nPosition: (${location.x.toFixed(2)}, ${location.y.toFixed(2)})`;
    tooltip += `\nHeading: ${(location.yaw * 180 / Math.PI).toFixed(1)}°`;
  }

  if (batteryLevel !== undefined) {
    tooltip += `\nBattery: ${batteryLevel.toFixed(0)}%`;
  }

  return tooltip;
};

/**
 * Get waypoint marker color based on type
 */
export const getWaypointColor = (
  isPickup: boolean,
  isDropoff: boolean
): string => {
  if (isPickup) return 'var(--color-success-500)';
  if (isDropoff) return 'var(--color-secondary-500)';
  return 'var(--color-warning-500)';
};

/**
 * Generate gradient for trajectory based on conflict status
 */
export const getTrajectoryGradient = (hasConflict: boolean): string => {
  if (hasConflict) {
    return 'linear-gradient(90deg, var(--color-error-500) 0%, var(--color-warning-500) 100%)';
  }
  return 'linear-gradient(90deg, var(--color-success-500) 0%, var(--color-primary-500) 100%)';
};

/**
 * Format distance for human readability
 */
export const formatDistance = (meters: number): string => {
  if (meters < 1) {
    return `${(meters * 100).toFixed(0)} cm`;
  }
  if (meters < 1000) {
    return `${meters.toFixed(1)} m`;
  }
  return `${(meters / 1000).toFixed(2)} km`;
};

/**
 * Format duration for human readability
 */
export const formatDuration = (seconds: number): string => {
  if (seconds < 60) {
    return `${seconds.toFixed(0)}s`;
  }
  if (seconds < 3600) {
    const minutes = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${minutes}m ${secs}s`;
  }
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  return `${hours}h ${minutes}m`;
};
