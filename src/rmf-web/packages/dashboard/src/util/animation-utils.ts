/**
 * Animation Utilities
 * Helper functions and hooks for smooth animations
 */

import { useEffect, useRef, useState } from 'react';

/**
 * Easing functions for smooth animations
 */
export const easing = {
  // Standard easing
  linear: (t: number) => t,
  easeIn: (t: number) => t * t,
  easeOut: (t: number) => t * (2 - t),
  easeInOut: (t: number) => (t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t),

  // Cubic easing
  easeInCubic: (t: number) => t * t * t,
  easeOutCubic: (t: number) => --t * t * t + 1,
  easeInOutCubic: (t: number) =>
    t < 0.5 ? 4 * t * t * t : (t - 1) * (2 * t - 2) * (2 * t - 2) + 1,

  // Elastic easing
  easeOutElastic: (t: number) => {
    const p = 0.3;
    return Math.pow(2, -10 * t) * Math.sin(((t - p / 4) * (2 * Math.PI)) / p) + 1;
  },

  // Bounce easing
  easeOutBounce: (t: number) => {
    if (t < 1 / 2.75) {
      return 7.5625 * t * t;
    } else if (t < 2 / 2.75) {
      return 7.5625 * (t -= 1.5 / 2.75) * t + 0.75;
    } else if (t < 2.5 / 2.75) {
      return 7.5625 * (t -= 2.25 / 2.75) * t + 0.9375;
    } else {
      return 7.5625 * (t -= 2.625 / 2.75) * t + 0.984375;
    }
  },
};

/**
 * Spring physics for natural motion
 */
export interface SpringConfig {
  stiffness?: number;
  damping?: number;
  mass?: number;
}

export const spring = (
  from: number,
  to: number,
  velocity: number,
  config: SpringConfig = {}
): { position: number; velocity: number } => {
  const { stiffness = 170, damping = 26, mass = 1 } = config;

  const dt = 1 / 60; // 60fps
  const force = -stiffness * (from - to);
  const dampingForce = -damping * velocity;
  const acceleration = (force + dampingForce) / mass;

  const newVelocity = velocity + acceleration * dt;
  const newPosition = from + newVelocity * dt;

  return { position: newPosition, velocity: newVelocity };
};

/**
 * Hook for intersection observer (scroll animations)
 */
export const useIntersectionObserver = (
  options: IntersectionObserverInit = {}
): [React.RefObject<HTMLDivElement>, boolean] => {
  const ref = useRef<HTMLDivElement>(null);
  const [isIntersecting, setIsIntersecting] = useState(false);

  useEffect(() => {
    const element = ref.current;
    if (!element) return;

    const observer = new IntersectionObserver(([entry]) => {
      setIsIntersecting(entry.isIntersecting);
    }, options);

    observer.observe(element);

    return () => {
      observer.disconnect();
    };
  }, [options]);

  return [ref, isIntersecting];
};

/**
 * Hook for animated value transitions
 */
export const useAnimatedValue = (
  targetValue: number,
  duration: number = 300,
  easingFn: (t: number) => number = easing.easeOutCubic
): number => {
  const [currentValue, setCurrentValue] = useState(targetValue);
  const startValueRef = useRef(targetValue);
  const startTimeRef = useRef<number | null>(null);

  useEffect(() => {
    startValueRef.current = currentValue;
    startTimeRef.current = null;

    let animationFrameId: number;

    const animate = (timestamp: number) => {
      if (startTimeRef.current === null) {
        startTimeRef.current = timestamp;
      }

      const elapsed = timestamp - startTimeRef.current;
      const progress = Math.min(elapsed / duration, 1);
      const easedProgress = easingFn(progress);

      const newValue =
        startValueRef.current + (targetValue - startValueRef.current) * easedProgress;

      setCurrentValue(newValue);

      if (progress < 1) {
        animationFrameId = requestAnimationFrame(animate);
      }
    };

    animationFrameId = requestAnimationFrame(animate);

    return () => {
      cancelAnimationFrame(animationFrameId);
    };
  }, [targetValue, duration, easingFn]);

  return currentValue;
};

/**
 * Hook for staggered animations
 */
export const useStaggeredAnimation = (
  count: number,
  delay: number = 50
): boolean[] => {
  const [visible, setVisible] = useState<boolean[]>(new Array(count).fill(false));

  useEffect(() => {
    const timeouts: NodeJS.Timeout[] = [];

    for (let i = 0; i < count; i++) {
      const timeout = setTimeout(() => {
        setVisible((prev) => {
          const next = [...prev];
          next[i] = true;
          return next;
        });
      }, i * delay);

      timeouts.push(timeout);
    }

    return () => {
      timeouts.forEach((timeout) => clearTimeout(timeout));
    };
  }, [count, delay]);

  return visible;
};

/**
 * Performance-optimized requestAnimationFrame wrapper
 */
export const useAnimationFrame = (callback: (deltaTime: number) => void) => {
  const requestRef = useRef<number>();
  const previousTimeRef = useRef<number>();

  useEffect(() => {
    const animate = (time: number) => {
      if (previousTimeRef.current !== undefined) {
        const deltaTime = time - previousTimeRef.current;
        callback(deltaTime);
      }
      previousTimeRef.current = time;
      requestRef.current = requestAnimationFrame(animate);
    };

    requestRef.current = requestAnimationFrame(animate);

    return () => {
      if (requestRef.current) {
        cancelAnimationFrame(requestRef.current);
      }
    };
  }, [callback]);
};

/**
 * Hook for hover state with delay
 */
export const useHoverWithDelay = (
  delay: number = 200
): [boolean, { onMouseEnter: () => void; onMouseLeave: () => void }] => {
  const [isHovered, setIsHovered] = useState(false);
  const timeoutRef = useRef<NodeJS.Timeout>();

  const onMouseEnter = () => {
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
    }
    timeoutRef.current = setTimeout(() => {
      setIsHovered(true);
    }, delay);
  };

  const onMouseLeave = () => {
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
    }
    setIsHovered(false);
  };

  useEffect(() => {
    return () => {
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
    };
  }, []);

  return [isHovered, { onMouseEnter, onMouseLeave }];
};

/**
 * Debounce hook for performance optimization
 */
export const useDebounce = <T,>(value: T, delay: number): T => {
  const [debouncedValue, setDebouncedValue] = useState<T>(value);

  useEffect(() => {
    const handler = setTimeout(() => {
      setDebouncedValue(value);
    }, delay);

    return () => {
      clearTimeout(handler);
    };
  }, [value, delay]);

  return debouncedValue;
};

/**
 * Throttle hook for performance optimization
 */
export const useThrottle = <T,>(value: T, interval: number): T => {
  const [throttledValue, setThrottledValue] = useState<T>(value);
  const lastUpdated = useRef<number>(Date.now());

  useEffect(() => {
    const now = Date.now();

    if (now >= lastUpdated.current + interval) {
      lastUpdated.current = now;
      setThrottledValue(value);
    } else {
      const timeout = setTimeout(() => {
        lastUpdated.current = Date.now();
        setThrottledValue(value);
      }, interval - (now - lastUpdated.current));

      return () => clearTimeout(timeout);
    }
  }, [value, interval]);

  return throttledValue;
};
