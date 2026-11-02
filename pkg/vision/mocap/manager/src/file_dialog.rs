use common::errors::*;
use mocap_proto::mocap::*;
use executor::channel::oneshot;

/// Native file pickers provided by the host application (e.g. the desktop app's webview).
///
/// Implementations are allowed to block until the user closes the dialog.
pub trait FileDialogs: Send + Sync + 'static {
    /// Returns the path picked by the user or None if the dialog was cancelled.
    fn open_file(&self, title: &str) -> Result<Option<String>>;

    /// Returns the path picked by the user or None if the dialog was cancelled.
    fn save_file(&self, title: &str, default_name: &str) -> Result<Option<String>>;
}

/// Runs a blocking dialog call on a separate thread so that we don't stall the executor.
pub(crate) async fn run_dialog<F: FnOnce() -> Result<Option<String>> + Send + 'static>(
    f: F
) -> Result<Option<String>> {
    let (sender, receiver) = oneshot::channel();
    std::thread::spawn(move || {
        let _ = sender.send(f());
    });

    receiver.recv().await.map_err(|_| err_msg("File dialog thread exited"))?
}

/// Copies over only the camera settings that are user adjustable in the UI.
///
/// Other fields like the blob_filter are intentionally left alone so that they
/// keep coming from the base config file.
pub(crate) fn copy_camera_settings(dst: &mut MocapCameraConfigureRequest, src: &MocapCameraConfigureRequest) {
    dst.set_frame_rate(src.frame_rate());
    dst.set_exposure_micros(src.exposure_micros());
    dst.set_pixel_threshold(src.pixel_threshold());
    dst.set_strobe_power(src.strobe_power());
    dst.set_leds_on(src.leds_on());
    dst.set_mjpeg(src.mjpeg().clone());
    dst.set_camera_controls(src.camera_controls().clone());
}
