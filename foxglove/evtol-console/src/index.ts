import type {ExtensionContext,PanelExtensionContext} from '@foxglove/extension';
import {mountConsole} from '../../../src/evtol_console/evtol_console/web/app.js';

function initPanel(context:PanelExtensionContext):()=>void {
  context.panelElement.style.height='100%';
  context.panelElement.style.overflow='hidden';
  const transport=async(request:unknown):Promise<any>=>{
    if(!context.callService)throw new Error('当前数据源不支持服务调用，请连接控制台启动的 Foxglove WebSocket');
    return await context.callService('/simulation/console',request as Record<string,unknown>);
  };
  return mountConsole(context.panelElement,transport);
}

export function activate(context:ExtensionContext):void {
  context.registerPanel({name:'EVTOL Simulation Console',initPanel});
}
