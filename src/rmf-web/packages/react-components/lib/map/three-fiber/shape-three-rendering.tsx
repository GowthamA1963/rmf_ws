import { Circle, Html } from '@react-three/drei';
import React from 'react';

interface ShapeThreeRenderingProps {
  position: [number, number, number];
  color: string;
  text?: string;
  circleShape: boolean;
}

export const debounce = (callback: () => void, delay: number): (() => void) => {
  let timeoutId: number | null = null;

  return () => {
    if (timeoutId) {
      clearTimeout(timeoutId);
    }

    timeoutId = window.setTimeout(() => {
      callback();
    }, delay);
  };
};

export const ShapeThreeRendering = ({
  position,
  color,
  text,
  circleShape,
}: ShapeThreeRenderingProps): JSX.Element => {
  const HEIGHT = 8;
  const ELEVATION = 0;
  const positionZ = HEIGHT / 2 + ELEVATION;
  const [isHovered, setIsHovered] = React.useState(false);

  const debouncedHandlePointerOver = debounce(() => {
    setIsHovered(true);
  }, 300);

  const debouncedHandlePointerOut = debounce(() => {
    setIsHovered(false);
  }, 300);

  const scaleFactor = isHovered ? 2 : 1.0;

  return (
    <>
      {circleShape ? (
        <Circle args={[0.6, 64]} position={[position[0], position[1], positionZ]}>
          <meshBasicMaterial color={color} />
        </Circle>
      ) : (
        <group position={position}>
          <mesh
            position={[0, 0, positionZ]}
            scale={[1.0, 1.0, 1.0]}
            onPointerOver={debouncedHandlePointerOver}
            onPointerOut={debouncedHandlePointerOut}
          >
            {isHovered && (
              <Html zIndexRange={[1]}>
                <div
                  style={{
                    backgroundColor: 'rgba(0, 0, 0, 0.85)',
                    color: '#ffffff',
                    padding: '0.4rem 0.7rem',
                    borderRadius: '6px',
                    fontSize: '1.1rem',
                    fontWeight: '600',
                    transform: `scale(${scaleFactor})`,
                    transition: 'transform 0.3s',
                    whiteSpace: 'nowrap',
                    border: '1px solid rgba(255, 255, 255, 0.3)',
                  }}
                >
                  {text}
                </div>
              </Html>
            )}
            <boxGeometry args={[2.5, 2.5, 2.5]} />
            <meshStandardMaterial color={color} opacity={0.7} transparent />
          </mesh>
        </group>
      )}
    </>
  );
};
