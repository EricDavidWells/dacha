use std::sync::Arc;
use std::collections::HashMap;
use std::time::Instant;

use common::bytes::Bytes;
use common::hash::FastHasherBuilder;
use common::errors::*;
use webview::*;
use mocap_proto::mocap::*;
use mocap_manager::*;
use mocap_manager::side_channel::*;
use file::{project_path, LocalPath, LocalPathBuf};
use executor::channel::spsc;
use executor::sync::SyncMutex;
use image::format::qoi::QOIDecoder;


use crate::protocol::*;
use crate::config::*;
use crate::http_server::*;

#[derive(Args)]
pub struct MocapAppArgs {
    data_dir: Option<LocalPathBuf>,

    #[arg(default = false)]
    enable_devtools: bool, 
}

pub struct MocapApp {
    shared: Arc<Shared>
}

/*
TODOs:
- Give the app a secret key and the port it is running on.
*/

struct Shared {
    http_server: AppHttpServer,
}

impl MocapApp {

    pub async fn create(args: MocapAppArgs) -> Result<()> {
        let data_dir = match args.data_dir {
            Some(v) => v,
            None => file::local_app_data_dir("mocap")?
        };

        file::create_dir_all(&data_dir).await?;

        let config = read_base_config(&data_dir).await?;

        let manager = Arc::new(MocapManager::create(
            config,
            data_dir.clone(),
        ).await?);

        let file_dialogs = Arc::new(WebViewFileDialogs::default());
        manager.set_file_dialogs(file_dialogs.clone()).await?;

        let side_channel = Arc::new(DataSideChannel::create());
        manager.set_side_channel(side_channel.clone()).await?;

        let http_server = AppHttpServer::create(&data_dir, manager.to_service(), side_channel).await?;
        let http_port = http_server.port();

        let shared = Arc::new(Shared {
            http_server,
        });

        /*
        TODO
        pub fn with_cookie(mut self, name: &str, value: &str, domain: &str, path: &str, http_only: bool, secure: bool) -> Self {
        */

        // TODO: Should also exit on internal manager failures.

        let mut builder = WebViewBuilder::new("Mocap Manager", 1920, 1080)
            .load_url(&format!("http://127.0.0.1:{}", http_port))
            .with_icon(Self::load_icon().await?)
            .with_prefer_dark_theme(true)
            .with_user_data_dir(data_dir.join("webview").to_str().unwrap())
            .on_init(move |handle| {
                let _ = file_dialogs.handle.apply(|v| *v = Some(handle));
            });

        if args.enable_devtools {
            builder = builder
                .with_devtools(true)
                .with_devtools_auto_open(true);
        }

        builder.run()?;

        Ok(())
    }

    async fn load_icon() -> Result<webview::Icon> {
        let data = file::read_asset("out/mocap_app/icons/icon.qoi").await?;
        let image = QOIDecoder::new().decode(&data)?;

        Ok(webview::Icon {
            width: image.width() as u32,
            height: image.height() as u32,
            rgba: image.array.data
        })
    }
}

/// Exposes the webview's native file pickers to the manager.
#[derive(Default)]
struct WebViewFileDialogs {
    // Set once the webview window has been created.
    handle: SyncMutex<Option<WebViewHandle>>,
}

impl WebViewFileDialogs {
    fn get_handle(&self) -> Result<WebViewHandle> {
        self.handle.apply(|v| v.clone())
            .map_err(|_| err_msg("Webview handle lock poisoned"))?
            .ok_or_else(|| err_msg("Webview not initialized yet"))
    }
}

impl FileDialogs for WebViewFileDialogs {
    fn open_file(&self, title: &str) -> Result<Option<String>> {
        self.get_handle()?.open_file_dialog(title)
    }

    fn save_file(&self, title: &str, default_name: &str) -> Result<Option<String>> {
        self.get_handle()?.save_file_dialog(title, Some(default_name))
    }
}
