设备协议栈
=========================

CORE
-----------------

端点结构体
""""""""""""""""""""""""""""""""""""

端点结构体主要用于注册不同端点地址的中断完成回调函数。

.. code-block:: C

    typedef struct usbd_endpoint {
        usb_slist_t list;
        uint8_t ep_addr;
        usbd_endpoint_callback ep_cb;
    } usbd_endpoint_t;

- **list** 端点的链表节点
- **ep_addr** 端点地址（带方向）
- **ep_cb** 端点中断回调函数

接口结构体
""""""""""""""""""""""""""""""""""""

接口结构体主要用于注册不同类设备除了标准设备请求外的其他请求，包括类设备请求、厂商设备请求和自定义设备请求。以及协议栈中的相关通知回调函数。

.. code-block:: C

    typedef struct usbd_interface {
        usb_slist_t list;
        /** Handler for USB Class specific commands*/
        usbd_request_handler class_handler;
        /** Handler for USB Vendor specific commands */
        usbd_request_handler vendor_handler;
        /** Handler for USB custom specific commands */
        usbd_request_handler custom_handler;
        /** Handler for USB event notify commands */
        usbd_notify_handler notify_handler;
        uint8_t intf_num;
        usb_slist_t ep_list;
    } usbd_interface_t;

- **list** 接口的链表节点
- **class_handler** class setup 请求回调函数
- **vendor_handler** vendor setup 请求回调函数
- **custom_handler** custom setup 请求回调函数
- **notify_handler** 中断标志、协议栈相关状态回调函数
- **intf_num** 当前接口偏移
- **ep_list** 端点的链表节点

类结构体
""""""""""""""""""""""""""""""""""""

类结构体主要用于挂载接口链表。后期可能会删除，因为这个部分跟接口其实是有关系的。

.. code-block:: C

    typedef struct usbd_class {
        usb_slist_t list;
        const char *name;
        usb_slist_t intf_list;
    } usbd_class_t;

- **list** 类的链表节点
- **name** 类的名称
- **intf_list** 接口的链表节点

usbd_event_notify_handler
""""""""""""""""""""""""""""""""""""

``usbd_event_notify_handler`` 是 USB 中断中的核心，用于处理不同的中断标志。包括复位、端点0 IN/OUT/SETUP、其他端点 IN/OUT 、SUSPEND、RESUME、SOF 中断等等。用户需要在 porting 接口中的 USB中断中调用该接口。

.. code-block:: C

    void usbd_event_notify_handler(uint8_t event, void *arg);

- **event**  中断事件
- **arg**  端点号

其中 ``event`` 有如下类型：

.. code-block:: C

    enum usbd_event_type {
        /** USB error reported by the controller */
        USBD_EVENT_ERROR,
        /** USB reset */
        USBD_EVENT_RESET,
        /** Start of Frame received */
        USBD_EVENT_SOF,
        /** USB connection established, hardware enumeration is completed */
        USBD_EVENT_CONNECTED,
        /** USB configuration done */
        USBD_EVENT_CONFIGURED,
        /** USB connection suspended by the HOST */
        USBD_EVENT_SUSPEND,
        /** USB connection lost */
        USBD_EVENT_DISCONNECTED,
        /** USB connection resumed by the HOST */
        USBD_EVENT_RESUME,

        /** USB interface selected */
        USBD_EVENT_SET_INTERFACE,
        /** USB interface selected */
        USBD_EVENT_SET_REMOTE_WAKEUP,
        /** USB interface selected */
        USBD_EVENT_CLEAR_REMOTE_WAKEUP,
        /** Set Feature ENDPOINT_HALT received */
        USBD_EVENT_SET_HALT,
        /** Clear Feature ENDPOINT_HALT received */
        USBD_EVENT_CLEAR_HALT,
        /** setup packet received */
        USBD_EVENT_SETUP_NOTIFY,
        /** ep0 in packet received */
        USBD_EVENT_EP0_IN_NOTIFY,
        /** ep0 out packet received */
        USBD_EVENT_EP0_OUT_NOTIFY,
        /** ep in packet except ep0 received */
        USBD_EVENT_EP_IN_NOTIFY,
        /** ep out packet except ep0 received */
        USBD_EVENT_EP_OUT_NOTIFY,
        /** Initial USB connection status */
        USBD_EVENT_UNKNOWN
    };

usbd_desc_register
""""""""""""""""""""""""""""""""""""

``usbd_desc_register`` 用来注册 USB 描述符，描述符种类包括：设备描述符、配置描述符（包含配置描述符、接口描述符、class 类描述符、端点描述符）、字符串描述符、设备限定描述符。

.. code-block:: C

    void usbd_desc_register(const uint8_t *desc);

- **desc**  描述符的句柄

usbd_msosv1_desc_register
""""""""""""""""""""""""""""""""""""

``usbd_msosv1_desc_register`` 用来注册一个 WINUSB 1.0 描述符。

.. code-block:: C

    void usbd_msosv1_desc_register(struct usb_msosv1_descriptor *desc);

- **desc**  描述符句柄

usbd_msosv2_desc_register
""""""""""""""""""""""""""""""""""""

``usbd_msosv2_desc_register`` 用来注册一个 WINUSB 2.0 描述符。

.. code-block:: C

    void usbd_msosv2_desc_register(struct usb_msosv2_descriptor *desc);

- **desc**  描述符句柄

usbd_bos_desc_register
""""""""""""""""""""""""""""""""""""

``usbd_bos_desc_register`` 用来注册一个 BOS 描述符， USB 2.1 版本以上必须注册。

.. code-block:: C

    void usbd_bos_desc_register(struct usb_bos_descriptor *desc);

- **desc**  描述符句柄

usbd_class_register
""""""""""""""""""""""""""""""""""""

``usbd_class_register`` 用来注册一个 class，该 class 中的接口链表成员，用于后续挂载多个接口。

.. code-block:: C

    void usbd_class_register(usbd_class_t *devclass);

- **devclass**  USB 设备类的句柄

usbd_class_add_interface
""""""""""""""""""""""""""""""""""""

``usbd_class_add_interface`` 用来给 USB 设备类增加接口，并将接口信息挂载在类的链表上。

.. code-block:: C

    void usbd_class_add_interface(usbd_class_t *devclass, usbd_interface_t *intf);

- **devclass**  USB 设备类的句柄
- **intf**   USB 设备接口的句柄

**usbd_interface_add_endpoint**
""""""""""""""""""""""""""""""""""""

``usbd_interface_add_endpoint`` 用来给 USB 接口增加端点，并将端点信息挂载在接口的链表上。

.. code-block:: C

    void usbd_interface_add_endpoint(usbd_interface_t *intf, usbd_endpoint_t *ep);


- **intf**  USB 设备接口的句柄
- **ep**    USB 设备端点的句柄

**usb_device_is_configured**
""""""""""""""""""""""""""""""""""""

``usb_device_is_configured`` 用来检查 USB 设备是否被配置（枚举）。

.. code-block:: C

    bool usb_device_is_configured(void);

- **return** 配置状态， 0 表示未配置， 1 表示配置成功

CDC ACM
-----------------

usbd_cdc_add_acm_interface
""""""""""""""""""""""""""""""""""""

``usbd_cdc_add_acm_interface`` 用来给 USB CDC ACM 类添加接口，并实现该接口相关的函数：

- ``cdc_acm_class_request_handler`` 用来处理 USB CDC ACM 类 Setup 请求。
- ``cdc_notify_handler`` 用来处理 USB CDC 其他中断回调函数。

.. code-block:: C

    void usbd_cdc_add_acm_interface(usbd_class_t *devclass, usbd_interface_t *intf);

- **devclass** 类的句柄
- **intf**  接口句柄

usbd_cdc_acm_set_line_coding
""""""""""""""""""""""""""""""""""""

``usbd_cdc_acm_set_line_coding`` 用来对串口进行配置，如果仅使用 USB 而不用 串口，该接口不用用户实现，使用默认。

.. code-block:: C

    void usbd_cdc_acm_set_line_coding(uint32_t baudrate, uint8_t databits, uint8_t parity, uint8_t stopbits);

- **baudrate** 波特率
- **databits**  数据位
- **parity**  校验位
- **stopbits**  停止位

usbd_cdc_acm_set_dtr
""""""""""""""""""""""""""""""""""""

``usbd_cdc_acm_set_dtr`` 用来控制串口 DTR 。如果仅使用 USB 而不用 串口，该接口不用用户实现，使用默认。

.. code-block:: C

    void usbd_cdc_acm_set_dtr(bool dtr);

- **dtr** dtr 为1表示拉低电平，为0表示拉高电平

usbd_cdc_acm_set_rts
""""""""""""""""""""""""""""""""""""

``usbd_cdc_acm_set_rts`` 用来控制串口 RTS 。如果仅使用 USB 而不用 串口，该接口不用用户实现，使用默认。

.. code-block:: C

    void usbd_cdc_acm_set_rts(bool rts);

- **rts** rts 为1表示拉低电平，为0表示拉高电平

HID
-----------------

usbd_hid_add_interface
""""""""""""""""""""""""""""""""""""

``usbd_hid_add_interface`` 用来给 USB HID 类添加接口，并实现该接口相关的函数：

- ``hid_class_request_handler`` 用来处理 USB HID 类的 Setup 请求。
- ``hid_custom_request_handler`` 用来处理 USB HID 获取报告描述符请求。
- ``hid_notify_handler`` 用来处理 USB HID 其他中断回调函数。

.. code-block:: C

    void usbd_hid_add_interface(usbd_class_t *devclass, usbd_interface_t *intf);

- **devclass** 类的句柄
- **intf**  接口句柄

usbd_hid_report_descriptor_register
""""""""""""""""""""""""""""""""""""""""""""

``usbd_hid_report_descriptor_register``  用来注册 hid 报告描述符。

.. code-block:: C

    void usbd_hid_report_descriptor_register(uint8_t intf_num, const uint8_t *desc, uint32_t desc_len);

- **intf_num** 当前 hid 报告描述符所在接口偏移
- **desc** 报告描述符
- **desc_len** 报告描述符长度

usbd_hid_set_request_callback
""""""""""""""""""""""""""""""""""""

``usbd_hid_set_request_callback``  用来注册 hid 类请求命令的回调函数。

.. code-block:: C

    void usbd_hid_set_request_callback( uint8_t intf_num,
                                        uint8_t (*get_report_callback)(uint8_t report_id, uint8_t report_type),
                                        void (*set_report_callback)(uint8_t report_id, uint8_t report_type, uint8_t *report, uint8_t report_len),
                                        uint8_t (*get_idle_callback)(uint8_t report_id),
                                        void (*set_idle_callback)(uint8_t report_id, uint8_t duration),
                                        void (*set_protocol_callback)(uint8_t protocol),
                                        uint8_t (*get_protocol_callback)(void));

- **intf_num** 当前 hid 报告描述符所在接口偏移
- **get_report_callback** get report命令处理回调函数
- **set_report_callback** set report命令处理回调函数
- **get_idle_callback** get idle命令处理回调函数
- **set_idle_callback** set idle命令处理回调函数
- **set_protocol_callback** set protocol命令处理回调函数
- **get_protocol_callback** get protocol命令处理回调函数

MSC
-----------------

usbd_msc_class_init
""""""""""""""""""""""""""""""""""""
``usbd_msc_class_init`` 用来给 MSC 类添加接口，并实现该接口相关函数，并且注册端点回调函数。（因为 msc bot 协议是固定的，所以不需要用于实现，因此端点回调函数自然不需要用户实现）。

- ``msc_storage_class_request_handler`` 用于处理 USB MSC Setup 中断请求。
- ``msc_storage_notify_handler`` 用于实现 USB MSC 其他中断回调函数。

- ``mass_storage_bulk_out`` 用于处理 USB MSC 端点 out 中断。
- ``mass_storage_bulk_in`` 用于处理 USB MSC 端点 in 中断。

.. code-block:: C

    void usbd_msc_class_init(uint8_t out_ep, uint8_t in_ep);

- **out_ep**     out 端点地址
- **in_ep**      in 端点地址

usbd_msc_get_cap
""""""""""""""""""""""""""""""""""""

``usbd_msc_get_cap`` 用来获取存储器的 lun、扇区个数和每个扇区大小。用户必须实现该函数。

.. code-block:: C

    void usbd_msc_get_cap(uint8_t lun, uint32_t *block_num, uint16_t *block_size);

- **lun** 存储逻辑单元，暂时无用，默认支持一个
- **block_num**  存储扇区个数
- **block_size**  存储扇区大小

usbd_msc_sector_read
""""""""""""""""""""""""""""""""""""

``usbd_msc_sector_read`` 用来对存储器某个扇区开始的地址进行数据读取。用户必须实现该函数。

.. code-block:: C

    int usbd_msc_sector_read(uint32_t sector, uint8_t *buffer, uint32_t length);

- **sector** 扇区偏移
- **buffer** 存储读取的数据的指针
- **length** 读取长度


usbd_msc_sector_write
""""""""""""""""""""""""""""""""""""

``usbd_msc_sector_write``  用来对存储器某个扇区开始写入数据。用户必须实现该函数。

.. code-block:: C

    int usbd_msc_sector_write(uint32_t sector, uint8_t *buffer, uint32_t length);

- **sector** 扇区偏移
- **buffer** 写入数据指针
- **length** 写入长度

UAC
-----------------

usbd_audio_add_interface
""""""""""""""""""""""""""""""""""""
``usbd_audio_add_interface``  用来给 USB Audio 类添加接口，并实现该接口相关的函数：

- ``audio_class_request_handler`` 用于处理 USB Audio Setup 中断请求。
- ``audio_notify_handler`` 用于实现 USB Audio 其他中断回调函数。

.. code-block:: C

    void usbd_audio_add_interface(usbd_class_t *devclass, usbd_interface_t *intf);

- **class** 类的句柄
- **intf**  接口句柄

usbd_audio_set_mute
""""""""""""""""""""""""""""""""""""

``usbd_audio_set_mute``  用来设置静音。

.. code-block:: C

    void usbd_audio_set_mute(uint8_t mute);

- **mute** 为1 表示静音，0相反

usbd_audio_set_volume
""""""""""""""""""""""""""""""""""""

``usbd_audio_set_volume``  用来设置音量。

.. code-block:: C

    void usbd_audio_set_volume(uint8_t vol);

- **vol** 音量，从 0-100

usbd_audio_set_interface_callback
""""""""""""""""""""""""""""""""""""

``usbd_audio_set_interface_callback``  用来开关音频数据传输。

.. code-block:: C

    void usbd_audio_set_interface_callback(uint8_t value);

- **value** 为1 表示开启 stream 传输，为0 相反

UVC
-----------------

usbd_video_add_interface
""""""""""""""""""""""""""""""""""""
``usbd_video_add_interface``  用来给 USB Video 类添加接口，并实现该接口相关的函数：

- ``video_class_request_handler`` 用于处理 USB Video Setup 中断请求。
- ``video_notify_handler`` 用于实现 USB Video 其他中断回调函数。

.. code-block:: C

    void usbd_video_add_interface(usbd_class_t *devclass, usbd_interface_t *intf);

- **class** 类的句柄
- **intf**  接口句柄

usbd_video_probe_and_commit_controls_init
""""""""""""""""""""""""""""""""""""""""""""""""""""""""

``usbd_video_probe_and_commit_controls_init``  用来开关视频数据传输。

.. code-block:: C

    void usbd_video_probe_and_commit_controls_init(uint32_t dwFrameInterval, uint32_t dwMaxVideoFrameSize, uint32_t dwMaxPayloadTransferSize);

- **value** 为1 表示开启 stream 传输，为0 相反

usbd_video_mjpeg_payload_fill
""""""""""""""""""""""""""""""""""""

``usbd_video_mjpeg_payload_fill``  用来填充 mjpeg 到新的 buffer中，其中会对 mjpeg 数据按帧进行切分，切分大小由 ``dwMaxPayloadTransferSize`` 控制，并添加头部信息，当前头部字节数为 2。头部信息见 ``struct video_mjpeg_payload_header``

.. code-block:: C

    uint32_t usbd_video_mjpeg_payload_fill(uint8_t *input, uint32_t input_len, uint8_t *output, uint32_t *out_len);

- **input** mjpeg 格式的数据包，从 FFD8~FFD9结束
- **input_len** mjpeg数据包大小
- **output** 输出缓冲区
- **out_len** 输出实际要发送的长度大小
- **return** 返回 usb 按照 ``dwMaxPayloadTransferSize`` 大小要发多少帧

DFU
-----------------

PORTING
-----------------