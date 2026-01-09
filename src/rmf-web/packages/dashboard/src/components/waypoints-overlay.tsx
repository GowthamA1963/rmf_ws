import React from 'react';
import {
  fromRmfCoords,
  Place,
  SVGOverlay,
  SVGOverlayProps,
  useAutoScale,
  viewBoxFromLeafletBounds,
  WaypointMarker as WaypointMarker_,
  withLabel,
} from 'react-components';

// no need memo since waypoint doesn't have state and should never re-render.
const WaypointMarker = withLabel(WaypointMarker_);

export interface WaypointsOverlayProps extends Omit<SVGOverlayProps, 'viewBox'> {
  waypoints: Place[];
  hideLabels?: boolean;
}

export const WaypointsOverlay = React.memo(
  ({ waypoints, hideLabels = false, ...otherProps }: WaypointsOverlayProps): JSX.Element => {
    const viewBox = viewBoxFromLeafletBounds(otherProps.bounds);
    // Set the size of the waypoint. At least for now we don't want for this to change. We left this here in case we want for this to change in the future.
    const size = 0.25;
    const scale = useAutoScale(60);

    return (
      <SVGOverlay viewBox={viewBox} {...otherProps}>
        {waypoints.map((waypoint, idx) => {
          const [x, y] = fromRmfCoords([waypoint.vertex.x, waypoint.vertex.y]);
          const isPickup = !!waypoint.pickupHandler;
          const isDropoff = !!waypoint.dropoffHandler;

          // Color coding: green for pickup, purple for dropoff, amber for regular
          const waypointColor = isPickup ? '#10b981' : isDropoff ? '#8b5cf6' : '#f59e0b';

          return (
            <g key={idx} className="waypoint-marker">
              <WaypointMarker
                cx={x}
                cy={y}
                size={size}
                aria-label={waypoint.vertex.name}
                style={{
                  transform: `scale(${scale})`,
                  transformOrigin: `${x}px ${y}px`,
                  cursor: 'pointer',
                  transition: 'all 0.2s ease',
                }}
                fill={waypointColor}
                labelText={waypoint.vertex.name}
                labelSourceX={x}
                labelSourceY={y}
                labelSourceRadius={size / 2}
                hideLabel={hideLabels}
              />
            </g>
          );
        })}
      </SVGOverlay>
    );
  },
);
