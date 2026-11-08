//go:build js && wasm

package main

import "syscall/js"

func byID(id string) js.Value {
	return js.Global().Get("document").Call("getElementById", id)
}

func newEl(tag, class, text string) js.Value {
	el := js.Global().Get("document").Call("createElement", tag)
	if class != "" {
		el.Set("className", class)
	}
	if text != "" {
		el.Set("textContent", text)
	}
	return el
}

func on(el js.Value, event string, handler func(js.Value)) {
	el.Call("addEventListener", event, js.FuncOf(func(_ js.Value, args []js.Value) any {
		handler(args[0])
		return nil
	}))
}

func onClick(el js.Value, handler func(js.Value)) { on(el, "click", handler) }

func setClass(el js.Value, class string, enabled bool) {
	el.Get("classList").Call("toggle", class, enabled)
}

func setText(el js.Value, text string) {
	if el.Get("textContent").String() != text {
		el.Set("textContent", text)
	}
}

func clearChildren(el js.Value) {
	el.Set("textContent", "")
}
