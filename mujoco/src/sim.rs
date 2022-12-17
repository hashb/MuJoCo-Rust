use crate::{
    helpers::{extract_vector_float_f64, Local},
    Model, State,
};

pub struct Simulation {
    pub state: State,
    pub model: Model,
}

impl Simulation {
    pub fn new(model: Model) -> Self {
        let state = State::new(&model);
        Self { state, model }
    }

    /// Set control vector
    pub fn control(&self, control: &[f64]) {
        let mj_data = self.state.ptr;
        let raw_vec = unsafe { (*mj_data).ctrl };

        if control.len() != self.model.nu() {
            return;
        }

        #[allow(clippy::needless_range_loop)]
        for i in 0..self.model.nu() {
            unsafe {
                *raw_vec.add(i) = control[i];
            }
        }
    }

    /// Evaulate constraint forces ans sensors
    pub fn evaluate_sensors(&self) {
        unsafe {
            mujoco_rs_sys::no_render::mj_sensorPos(self.model.ptr(), self.state.ptr());
            mujoco_rs_sys::no_render::mj_sensorVel(self.model.ptr(), self.state.ptr());
            mujoco_rs_sys::no_render::mj_sensorAcc(self.model.ptr(), self.state.ptr());
        };
    }

    /// Advance simulation by one step
    pub fn step(&self) {
        unsafe {
            mujoco_rs_sys::no_render::mj_step(self.model.ptr(), self.state.ptr());
        };
    }

    /// Returns positions of bodies in inertial frame
    pub fn xpos(&self) -> Vec<[f64; 3]> {
        let mj_data = self.state.ptr();
        let raw_vec = unsafe { (*mj_data).xpos };

        let raw_xpos =
            extract_vector_float_f64(raw_vec as *mut Local<f64>, 3, self.model.ngeom());

        let mut xpos: Vec<[f64; 3]> = Vec::new();

        for i in 0..self.model.ngeom() {
            let entry: [f64; 3] =
                [raw_xpos[i * 3], raw_xpos[i * 3 + 1], raw_xpos[i * 3 + 2]];
            xpos.push(entry);
        }

        xpos
    }

    /// Returns rotations of bodies
    pub fn xquat(&self) -> Vec<[f64; 4]> {
        let mj_data = self.state.ptr();
        let raw_vec = unsafe { (*mj_data).xquat };
        let raw_quat =
            extract_vector_float_f64(raw_vec as *mut Local<f64>, 4, self.model.ngeom());
        let mut xquat: Vec<[f64; 4]> = Vec::new();

        for i in 0..self.model.ngeom() {
            let entry: [f64; 4] = [
                raw_quat[i * 4],
                raw_quat[i * 4 + 1],
                raw_quat[i * 4 + 2],
                raw_quat[i * 4 + 3],
            ];
            xquat.push(entry);
        }

        xquat
    }

    /// Returns generalized positions of bodies
    pub fn qpos(&self) -> Vec<f64> {
        let mj_data = self.state.ptr();
        let raw_vec = unsafe { (*mj_data).qpos };

        let mut qpos: Vec<f64> = Vec::new();

        for i in 0..self.model.nq() {
            let entry = unsafe { *raw_vec.add(i) };
            qpos.push(entry);
        }

        qpos
    }

    /// Returns generalized velocities of bodies
    pub fn qvel(&self) -> Vec<f64> {
        let mj_data = self.state.ptr();
        let raw_vec = unsafe { (*mj_data).qvel };

        let mut qvel: Vec<f64> = Vec::new();

        for i in 0..self.model.nv() {
            let entry = unsafe { *raw_vec.add(i) };
            qvel.push(entry);
        }

        qvel
    }

    /// Returns contracts
    pub fn cfrc_ext(&self) -> Vec<[f64; 6]> {
        let mj_data = self.state.ptr();
        let raw_vec = unsafe { (*mj_data).cfrc_ext };
        let raw_quat =
            extract_vector_float_f64(raw_vec as *mut Local<f64>, 6, self.model.nbody());
        let mut cfrc_ext: Vec<[f64; 6]> = Vec::new();

        for i in 0..self.model.nbody() {
            let entry: [f64; 6] = [
                raw_quat[i * 6],
                raw_quat[i * 6 + 1],
                raw_quat[i * 6 + 2],
                raw_quat[i * 6 + 3],
                raw_quat[i * 6 + 4],
                raw_quat[i * 6 + 5],
            ];
            cfrc_ext.push(entry);
        }

        cfrc_ext
    }

    /// Read sensor data
    pub fn sensordata(&self) -> Vec<f64> {
        let mj_data = self.state.ptr();
        let raw_vec = unsafe { (*mj_data).sensordata };
        let mut sensordata: Vec<f64> = Vec::new();

        for i in 0..self.model.nsensordata() {
            sensordata.push(unsafe { *raw_vec.add(i) });
        }

        sensordata
    }

    /// Resets the state to the default values
    pub fn reset(&mut self) {
        unsafe {
            mujoco_rs_sys::no_render::mj_resetData(self.model.ptr(), self.state.ptr())
        }
    }
}
