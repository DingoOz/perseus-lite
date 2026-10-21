// builtin_interfaces / std_msgs types
export interface TimeType {
  sec: number;
  nanosec: number;
}

export interface HeaderType {
  stamp?: TimeType; // rosbridge does NOT fill this in; leaving it out publishes a zero stamp
  frame_id?: string;
}

export interface Float64MultiArrayType {
  data: Array<number>;
}

// geometry_msgs types
export interface Vector3Type {
  x: number;
  y: number;
  z: number;
}

export interface TwistType {
  linear: Vector3Type;
  angular: Vector3Type;
}

export interface TwistStampedType {
  header: HeaderType;
  twist: TwistType;
}

// Service types
export interface EmptyRequestType {}

export interface EmptyResponseType {}

export interface SetBoolRequestType {
  data: boolean;
}

export interface SetBoolResponseType {
  success: boolean;
  message: string;
}
