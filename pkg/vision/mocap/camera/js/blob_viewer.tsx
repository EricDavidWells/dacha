import React from "react";
import { orientation_transform, apply_orientation_transform } from "./orientation";


export class Blob2dViewer extends React.Component<{ status: any, results: any, rotation: number }> {

    _canvas: React.RefObject<HTMLCanvasElement> = React.createRef();

    state = {
        canvas_width: undefined,
        canvas_height: undefined,
    }

    componentDidMount() {
        this.componentDidUpdate();
    }

    componentDidUpdate() {
        let canvas = this._canvas.current;
        if (!canvas) {
            return;
        }

        let container = canvas.parentElement;
        if (!container) {
            return;
        }

        let status = this.props.status;

        let display = orientation_transform(this.props.rotation || 0, status.sensor.frame_width, status.sensor.frame_height);

        let rect = container.getBoundingClientRect();
        let canvas_width = Math.ceil(rect.width);
        let canvas_height = Math.round((display.height / display.width) * canvas_width);

        if (canvas_width != this.state.canvas_width || canvas_height != this.state.canvas_height) {
            this.setState({
                canvas_height,
                canvas_width
            });
            return;
        }

        let ctx = canvas.getContext('2d');
        if (!ctx) {
            return;
        }

        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, canvas_width, canvas_height);

        if (!this.props.results) {
            return;
        }

        let blobs = this.props.results.results.blobs || [];

        let scale = canvas_width / display.width;

        blobs.map((blob) => {

            // Note that we draw it from the user perspective looking at the camera from in front of it.
            let [x, y] = apply_orientation_transform(display.matrix, blob.x, blob.y);

            ctx.fillStyle = '#fff';
            ctx.beginPath();
            // TODO: Verify 0.5, 0.5 is the center of a pixel.
            ctx.arc(x * scale, y * scale, blob.radius_a * scale, 0, 2 * Math.PI);
            ctx.fill();
        });
    }

    render() {
        return (
            <div style={{ width: '100%' }}>
                <canvas ref={this._canvas} width={this.state.canvas_width} height={this.state.canvas_height}></canvas>
            </div>
        )
    }
}
