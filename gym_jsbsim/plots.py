import pandas as pd

import holoviews as hv
from holoviews.operation import gridmatrix
hv.renderer('bokeh')

def generate_figures(log):
    log = pd.DataFrame(log).astype(float) # convert to float in case of string inputs

    # add a few more columns
    log['time'] = log.index
    log['position/h-desired-ft'] = log['position/h-sl-ft'] - log['error/altitude-error-ft']
    log['fcs/left-aileron-pos-norm-diff'] = log['fcs/left-aileron-pos-norm'] - log['fcs/aileron-cmd-norm']
    log['fcs/right-aileron-pos-norm-diff'] = log['fcs/right-aileron-pos-norm'] - log['fcs/aileron-cmd-norm']

    # create plots
    ## altitude
    h = hv.Curve(log[['time', 'position/h-sl-ft']], label='h-sl')
    h_desired = hv.Curve(log[['time', 'position/h-desired-ft']], label='h-desired')
    altitude = (h * h_desired).opts(hv.opts.Curve(tools=['hover']),
                                    hv.opts.Overlay(legend_position='top_left', ylabel='ft'))

    ## attitude
    pitch = hv.Curve(log[['time', 'attitude/pitch-rad']], label='pitch')
    roll = hv.Curve(log[['time', 'attitude/roll-rad']], label='roll')
    attitude = (pitch * roll).opts(hv.opts.Curve(tools=['hover']),
                                    hv.opts.Overlay(legend_position='top_left', ylabel='rad'))

    ## grid
    grid = gridmatrix(hv.Dataset(log[['velocities/u-fps', 'velocities/v-fps', 'velocities/w-fps', 'time']]),
                    chart_type=hv.Curve, diagonal_type=hv.Scatter)

    ## angular_vel
    pitch_dot = hv.Curve(log[['time', 'velocities/q-rad_sec']], label='pitch rate')
    roll_dot = hv.Curve(log[['time', 'velocities/p-rad_sec']], label='roll rate')
    yaw_dot = hv.Curve(log[['time', 'velocities/r-rad_sec']], label='yaw rate')

    angular_vel = (pitch_dot * roll_dot * yaw_dot).opts(
        hv.opts.Curve(tools=['hover']),
        hv.opts.Overlay(legend_position='top_left', ylabel='rad / sec'))

    # aileron
    left_ail = hv.Curve(log[['time', 'fcs/left-aileron-pos-norm']], label='left ail.')
    right_ail = hv.Curve(log[['time', 'fcs/right-aileron-pos-norm']], label='right ail.')
    cmd_ail = hv.Curve(log[['time', 'fcs/aileron-cmd-norm']], label='cmd ail.')

    aileron = (left_ail * right_ail * cmd_ail).opts(
        hv.opts.Curve(tools=['hover']),
        hv.opts.Overlay(legend_position='top_left', ylabel='normalised position [-1, 1]')
    )

    ## elevator
    elevator_pos = hv.Curve(log[['time', 'fcs/elevator-pos-norm']], label='elevator')
    elevator_cmd = hv.Curve(log[['time', 'fcs/elevator-cmd-norm']], label='cmd elevator.')

    elevator = (elevator_pos * elevator_cmd).opts(
        hv.opts.Curve(tools=['hover']),
        hv.opts.Overlay(legend_position='top_left', ylabel='normalised position [-1, 1]')
    )

    ## rudder
    rudder_pos = hv.Curve(log[['time', 'fcs/rudder-pos-norm']], label='rudder')
    rudder_cmd = hv.Curve(log[['time', 'fcs/rudder-cmd-norm']], label='cmd rudder.')

    rudder = (rudder_pos * rudder_cmd).opts(
        hv.opts.Curve(tools=['hover']),
        hv.opts.Overlay(legend_position='top_left', ylabel='normalised position [-1, 1]')
    )

    ## deg_errors
    sideslip = hv.Curve(log[['time', 'aero/beta-deg']], label='sideslip')
    track_error = hv.Curve(log[['time', 'error/track-error-deg']], label='track error')

    deg_errors = (sideslip * track_error).opts(
        hv.opts.Curve(tools=['hover']),
        hv.opts.Overlay(legend_position='top_left', ylabel='deg')
    )
        
    return dict(
        altitude=altitude,
        attitude=attitude,
        vel_grid=grid,
        angular_vel=angular_vel,
        aileron=aileron,
        elevator=elevator,
        rudder=rudder,
        deg_errors=deg_errors
    )
