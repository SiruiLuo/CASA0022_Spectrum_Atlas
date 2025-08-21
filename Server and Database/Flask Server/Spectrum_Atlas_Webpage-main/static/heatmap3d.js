// 3D热力图渲染器
class Heatmap3DRenderer {
    static instances = {};
    constructor(containerId) {
        // 若已有旧实例，先释放资源
        if (Heatmap3DRenderer.instances[containerId]) {
            Heatmap3DRenderer.instances[containerId].dispose();
        }

        this.container = document.getElementById(containerId);
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;
        this.modelGroup = new THREE.Group();
        this.heatmapMesh = null;
        this.barsGroup = null; // 新增：柱状体组合
        this.roomBoundary = null; // 新增：房间边界
        this.currentData = null;
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        this.tooltip = document.getElementById('heatmap-tooltip');
        this.lastMouseMoveTime = 0; // 新增：鼠标移动节流
        this.mouseMoveThrottle = 16; // 新增：约60fps的节流

        // 悬浮指示器
        this.hoverIndicator = null;
        
        // 记录实例
        Heatmap3DRenderer.instances[containerId] = this;

        this._animationId = null; // 用于取消 requestAnimationFrame

        this.init();
    }
    
    init() {
        // 创建场景
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0xffffff);
        
        // 创建相机
        this.camera = new THREE.PerspectiveCamera(
            75, 
            this.container.clientWidth / this.container.clientHeight, 
            0.1, 
            1000
        );
        this.camera.position.set(4, 4, 4);
        
        // 创建渲染器
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.container.appendChild(this.renderer.domElement);
        
        // 创建控制器
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.screenSpacePanning = false;
        this.controls.minDistance = 2;
        this.controls.maxDistance = 20;
        this.controls.maxPolarAngle = Math.PI / 2;
        
        // 添加光源
        this.addLights();
        
        // 添加网格
        this.addGrid();
        
        // 绑定事件
        this.bindEvents();
        
        // 开始渲染循环
        this.animate();
    }
    
    addLights() {
        // 环境光
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        this.scene.add(ambientLight);
        
        // 方向光
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        this.scene.add(directionalLight);
        
        // 点光源
        const pointLight = new THREE.PointLight(0xffffff, 0.5);
        pointLight.position.set(-5, 5, -5);
        this.scene.add(pointLight);
    }
    
    addGrid() {
        // 添加网格辅助线
        const gridHelper = new THREE.GridHelper(10, 10, 0x888888, 0xcccccc);
        this.scene.add(gridHelper);
        
        // 添加坐标轴
        const axesHelper = new THREE.AxesHelper(2);
        this.scene.add(axesHelper);
        // 将模型组加入场景
        this.scene.add(this.modelGroup);

        // 创建悬浮亮点（亮黄色方块）
        const hoverGeo = new THREE.BoxGeometry(0.1, 0.1, 0.1);
        const hoverMat = new THREE.MeshBasicMaterial({color:0xffff00});
        hoverMat.toneMapped = false; // 保持高亮
        this.hoverIndicator = new THREE.Mesh(hoverGeo, hoverMat);
        this.hoverIndicator.visible = false;
        this.scene.add(this.hoverIndicator);
    }
    
    // 新增：计算二维凸包
    computeConvexHull(points) {
        if (!points || points.length < 3) return points || [];
        // 使用单调链算法 (O(n log n))
        const pts = points.map(p => ({ x: p.x, z: p.z })).sort((a, b) => a.x === b.x ? a.z - b.z : a.x - b.x);
        const cross = (o, a, b) => (a.x - o.x) * (b.z - o.z) - (a.z - o.z) * (b.x - o.x);
        const lower = [];
        for (const p of pts) {
            while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) {
                lower.pop();
            }
            lower.push(p);
        }
        const upper = [];
        for (let i = pts.length - 1; i >= 0; i--) {
            const p = pts[i];
            while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) {
                upper.pop();
            }
            upper.push(p);
        }
        // 合并，最后一个点与第一个点重复，去掉
        upper.pop();
        lower.pop();
        return lower.concat(upper);
    }

    // 更新：沿栅格精确生成墙壁，完整包裹凹凸区域
    addRoomBoundary(points, gridW, gridH) {
        if (!points || points.length === 0 || !gridW || !gridH) return;

        // 移除旧边界
        if (this.roomBoundary) {
            this.scene.remove(this.roomBoundary);
            this.roomBoundary = null;
        }

        const wallHeight = 2;
        const wallThickness = 0.15; // 15cm，更厚墙体
        // 计算网格尺寸（单元格大小）
        const cellW = 10 / gridW;
        const cellH = 10 / gridH;

        // 构建占用矩阵
        const occ = Array.from({ length: gridH }, () => Array(gridW).fill(false));
        points.forEach(p => {
            const xi = Math.floor((p.x / 10 + 0.5) * gridW);
            const yi = Math.floor((p.z / 10 + 0.5) * gridH);
            if (xi >= 0 && xi < gridW && yi >= 0 && yi < gridH) {
                occ[yi][xi] = true;
            }
        });

        // --- 形态学闭运算：填补小间隙 -----------------
        const closeIter = 1; // 一次闭运算就足够
        for (let iter = 0; iter < closeIter; iter++) {
            const tmp = occ.map(row => [...row]);
            for (let y = 0; y < gridH; y++) {
                for (let x = 0; x < gridW; x++) {
                    if (tmp[y][x]) continue; // 该点已占用
                    let neigh = 0;
                    for (let dy = -1; dy <= 1; dy++) {
                        for (let dx = -1; dx <= 1; dx++) {
                            if (dx === 0 && dy === 0) continue;
                            const nx = x + dx, ny = y + dy;
                            if (nx >= 0 && nx < gridW && ny >= 0 && ny < gridH && tmp[ny][nx]) neigh++;
                        }
                    }
                    if (neigh >= 5) occ[y][x] = true; // 周围>=5占用则填充
                }
            }
        }
        // -------------------------------------------

        // flood fill 外部区域
        const outside = Array.from({ length: gridH }, () => Array(gridW).fill(false));
        const queue = [];
        // 边界空格入队
        for (let x = 0; x < gridW; x++) {
            if (!occ[0][x]) { outside[0][x] = true; queue.push([x,0]); }
            if (!occ[gridH-1][x]) { outside[gridH-1][x] = true; queue.push([x,gridH-1]); }
        }
        for (let y = 0; y < gridH; y++) {
            if (!occ[y][0]) { outside[y][0] = true; queue.push([0,y]); }
            if (!occ[y][gridW-1]) { outside[y][gridW-1] = true; queue.push([gridW-1,y]); }
        }

        const dirs = [[1,0],[-1,0],[0,1],[0,-1]];
        while (queue.length) {
            const [cx, cy] = queue.shift();
            for (const [dx, dy] of dirs) {
                const nx = cx + dx, ny = cy + dy;
                if (nx>=0 && nx<gridW && ny>=0 && ny<gridH && !occ[ny][nx] && !outside[ny][nx]) {
                    outside[ny][nx] = true;
                    queue.push([nx, ny]);
                }
            }
        }

        const wallMat = new THREE.MeshLambertMaterial({
            color: 0x888888,
            transparent: true,
            opacity: 0.8, // 更不透明
            depthWrite: true
        });

        this.roomBoundary = new THREE.Group();

        // 辅助函数：在给定中心、尺寸、方向创建墙壁
        const createWall = (cx, cz, length, horizontal) => {
            const geo = horizontal ? new THREE.BoxGeometry(length, wallHeight, wallThickness)
                                    : new THREE.BoxGeometry(wallThickness, wallHeight, length);
            const mesh = new THREE.Mesh(geo, wallMat);
            mesh.position.set(cx, wallHeight / 2, cz);
            mesh.receiveShadow = true;
            this.roomBoundary.add(mesh);
        };

        // 仅当邻接外部空格时才放墙
        for (let y = 0; y < gridH; y++) {
            for (let x = 0; x < gridW; x++) {
                if (!occ[y][x]) continue;

                const worldX = (x / gridW - 0.5) * 10;
                const worldZ = (y / gridH - 0.5) * 10;

                // West 边
                if (x === 0 || outside[y][x-1]) {
                    const cx = worldX - cellW / 2;
                    createWall(cx, worldZ, cellH, false);
                }
                // East 边
                if (x === gridW-1 || outside[y][x+1]) {
                    const cx = worldX + cellW / 2;
                    createWall(cx, worldZ, cellH, false);
                }
                // North 边
                if (y === gridH-1 || outside[y+1]?.[x]) {
                    const cz = worldZ + cellH / 2;
                    createWall(worldX, cz, cellW, true);
                }
                // South 边
                if (y === 0 || outside[y-1]?.[x]) {
                    const cz = worldZ - cellH / 2;
                    createWall(worldX, cz, cellW, true);
                }
            }
        }

        this.modelGroup.add(this.roomBoundary);
    }
    
    bindEvents() {
        // 记录绑定函数以便移除
        this._onMouseMove = (event)=>{
            const now = Date.now();
            if (now - this.lastMouseMoveTime > this.mouseMoveThrottle) {
                this.lastMouseMoveTime = now;
                this.onMouseMove(event);
            }
        };
        this._onResize = ()=>{ this.onWindowResize(); };

        // 事件监听
        this.renderer.domElement.addEventListener('mousemove', this._onMouseMove);
         
        window.addEventListener('resize', this._onResize);
    }
    
    onMouseMove(event) {
        const rect = this.renderer.domElement.getBoundingClientRect();
        this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
        
        // 检查与柱状体组合的交互
        this.raycaster.setFromCamera(this.mouse, this.camera);
        
        if (this.barsGroup) {
            const intersects = this.raycaster.intersectObjects(this.barsGroup.children);
            
            if (intersects.length > 0) {
                const intersect = intersects[0];
                const object = intersect.object;
                
                // 检查是否是柱状图对象
                if (object.userData && object.userData.rssi !== undefined) {
                    const rssi = object.userData.rssi;
                    const speedStr = this.computeSpeedMbps(rssi);
                    const worldX = object.userData.worldX;
                    const worldZ = object.userData.worldZ;
                    
                    this.showTooltip(event.clientX, event.clientY, worldX, worldZ, rssi, speedStr);

                    // 更新悬浮亮点位置并显示
                    const barHeight = (object.geometry?.parameters?.height) || 0.1;
                    this.hoverIndicator.position.set(object.position.x, barHeight + 0.05, object.position.z);
                    this.hoverIndicator.visible = true;
                    return;
                }
            }
        }
        
        this.hideTooltip();
        // 隐藏悬浮亮点
        if(this.hoverIndicator) this.hoverIndicator.visible = false;
    }
    
    showTooltip(mouseX, mouseY, worldX, worldY, rssi, speedStr) {
        this.tooltip.innerHTML = `
            <strong>Position:</strong> (${worldX.toFixed(2)}, ${worldY.toFixed(2)}) m<br>
            <strong>RSSI:</strong> ${rssi.toFixed(1)} dBm<br>
            <strong>Speed:</strong> ${speedStr}
        `;
        this.tooltip.style.left = mouseX + 'px';
        this.tooltip.style.top = mouseY + 'px';
        this.tooltip.classList.add('visible');
    }
    
    hideTooltip() {
        this.tooltip.classList.remove('visible');
    }
    
    onWindowResize() {
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }
    
    animate() {
        this._animationId = requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
    
    // 渲染热力图数据
    renderHeatmap(data) {
        this.currentData = data;
        
        // 清除并释放资源
        const toDispose = [this.barsGroup, this.heatmapMesh, this.roomBoundary, this.annotationGroup];
        toDispose.forEach(obj=>{
            if (obj) {
                this.disposeObject3D(obj);
                this.modelGroup.remove(obj);
            }
        });
        this.barsGroup = this.heatmapMesh = this.roomBoundary = this.annotationGroup = null;
        
        // 创建新的3D热力图
        this.createHeatmapMesh(data);
        
        // createHeatmapMesh 已在内部处理边界、barsGroup 及相机位置
    }
    
    createHeatmapMesh(data) {
        // 创建3D热力图网格
        const geometry = new THREE.PlaneGeometry(10, 10, data.width - 1, data.height - 1);
        // --- Heatmap plane material change ---
        const material = new THREE.MeshLambertMaterial({
            color: 0xffffff,
            transparent: false,
            opacity: 1,
            depthWrite: true,
            side: THREE.DoubleSide
        });
        
        // 创建基础网格
        this.heatmapMesh = new THREE.Mesh(geometry, material);
        this.heatmapMesh.rotation.x = -Math.PI / 2; // 旋转到水平面
        this.heatmapMesh.receiveShadow = true;
        this.scene.add(this.heatmapMesh);
        
        // 为每个有效数据点创建3D柱状图
        const validPoints = [];
        for (let y = 0; y < data.height; y++) {
            for (let x = 0; x < data.width; x++) {
                const rssi = data.data[y][x];
                if (rssi !== null && !isNaN(rssi)) {
                    validPoints.push({
                        x: (x / data.width - 0.5) * 10,
                        z: -(y / data.height - 0.5) * 10,
                        height: Math.max(0.1, (rssi - data.min_rssi) / (data.max_rssi - data.min_rssi) * 2),
                        rssi: rssi
                    });
                }
            }
        }
        
        // 创建柱状体组合以提高性能
        this.barsGroup = new THREE.Group();
        
        // 使用InstancedMesh来优化大量柱状体的渲染
        if (validPoints.length > 100) {
            // 对于大量数据点，使用InstancedMesh
            this.createInstancedBars(validPoints, data);
        } else {
            // 对于少量数据点，使用传统方法
            this.createIndividualBars(validPoints, data);
        }
        
        // 将房间边界添加到场景（基于凸包）
        this.addRoomBoundary(validPoints, data.width, data.height);
        
        this.modelGroup.add(this.barsGroup);
        
        // 调整相机位置
        this.camera.position.set(4, 4, 4);
        this.controls.target.set(0, 0, 0);
        this.controls.update();
    }
    
    // 新增：创建实例化柱状体（高性能）
    createInstancedBars(validPoints, data) {
        const barGeometry = new THREE.BoxGeometry(0.1, 1, 0.1);
        // --- Instanced bars material ---
        const barMaterial = new THREE.MeshLambertMaterial({
            transparent: false,
            opacity: 1,
            depthWrite: true
        });
        
        const instancedMesh = new THREE.InstancedMesh(barGeometry, barMaterial, validPoints.length);
        instancedMesh.castShadow = true;
        instancedMesh.receiveShadow = true;
        
        const matrix = new THREE.Matrix4();
        const color = new THREE.Color();
        
        validPoints.forEach((point, index) => {
            // 设置变换矩阵
            matrix.makeScale(1, point.height, 1);
            matrix.setPosition(point.x, point.height / 2, point.z);
            instancedMesh.setMatrixAt(index, matrix);
            
            // 设置颜色
            const heatmapColor = this.getHeatmapColor(point.rssi, data.min_rssi, data.max_rssi);
            color.setRGB(heatmapColor.r / 255, heatmapColor.g / 255, heatmapColor.b / 255);
            instancedMesh.setColorAt(index, color);
            
            // 存储用户数据（注意：InstancedMesh不支持userData，所以我们需要单独处理交互）
        });
        
        // 为交互创建单独的几何体
        this.createInteractionGeometry(validPoints, data);
        
        this.barsGroup.add(instancedMesh);
    }
    
    // 新增：创建传统柱状体（适用于少量数据点）
    createIndividualBars(validPoints, data) {
        validPoints.forEach(point => {
            const color = this.getHeatmapColor(point.rssi, data.min_rssi, data.max_rssi);
            const barGeometry = new THREE.BoxGeometry(0.1, point.height, 0.1);
            // --- Individual bars ---
            const barMaterial = new THREE.MeshLambertMaterial({
                color: new THREE.Color(color.r / 255, color.g / 255, color.b / 255),
                transparent: false,
                opacity: 1,
                depthWrite: true
            });
            
            const bar = new THREE.Mesh(barGeometry, barMaterial);
            bar.position.set(point.x, point.height / 2, point.z);
            bar.castShadow = true;
            bar.receiveShadow = true;
            
            // 存储RSSI值用于交互
            bar.userData = { rssi: point.rssi, worldX: point.x, worldZ: point.z };
            
            this.barsGroup.add(bar);
        });
    }
    
    // 新增：为InstancedMesh创建交互几何体
    createInteractionGeometry(validPoints, data) {
        // 为每个点创建小的交互几何体
        validPoints.forEach(point => {
            const interactionGeometry = new THREE.BoxGeometry(0.15, point.height, 0.15);
            const interactionMaterial = new THREE.MeshBasicMaterial({
                transparent: true,
                opacity: 0.01, // 几乎透明
                color: 0xffffff
            });
            
            const interactionMesh = new THREE.Mesh(interactionGeometry, interactionMaterial);
            interactionMesh.position.set(point.x, point.height / 2, point.z);
            interactionMesh.userData = { rssi: point.rssi, worldX: point.x, worldZ: point.z };
            
            this.barsGroup.add(interactionMesh);
        });
    }
    
    // 根据RSSI值获取颜色
    getHeatmapColor(value, min, max) {
        const normalized = (value - min) / (max - min);
        const clamped = Math.max(0, Math.min(1, normalized));
        
        // 使用彩虹色映射：红(强)→紫(弱)
        const hueDeg = (1 - clamped) * 270;
        const [rN, gN, bN] = this.hslToRgb(hueDeg / 360, 1, 0.5);
        
        return {
            r: Math.round(rN * 255),
            g: Math.round(gN * 255),
            b: Math.round(bN * 255)
        };
    }
    
    // HSL转RGB辅助函数
    hslToRgb(h, s, l) {
        let r, g, b;
        if (s === 0) {
            r = g = b = l;
        } else {
            const hue2rgb = (p, q, t) => {
                if (t < 0) t += 1;
                if (t > 1) t -= 1;
                if (t < 1 / 6) return p + (q - p) * 6 * t;
                if (t < 1 / 2) return q;
                if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6;
                return p;
            };
            const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
            const p = 2 * l - q;
            r = hue2rgb(p, q, h + 1 / 3);
            g = hue2rgb(p, q, h);
            b = hue2rgb(p, q, h - 1 / 3);
        }
        return [r, g, b];
    }
    
    // 递归释放几何体、材质、纹理
    disposeObject3D(obj) {
        if (!obj) return;
        obj.traverse((child)=>{
            if (child.geometry) {
                child.geometry.dispose();
            }
            if (child.material) {
                if (Array.isArray(child.material)) {
                    child.material.forEach(m=>m && m.dispose());
                } else {
                    child.material.dispose();
                }
            }
            if (child.texture) {
                child.texture.dispose();
            }
        });
    }
    
    // 清理资源
    dispose() {
        // 防止重复调用
        if (this._disposed) return;
        this._disposed = true;

        // 取消动画循环
        if (this._animationId !== null) {
            cancelAnimationFrame(this._animationId);
        }

        // 移除事件监听
        if (this.renderer && this._onMouseMove) {
            this.renderer.domElement.removeEventListener('mousemove', this._onMouseMove);
        }
        if (this._onResize) {
            window.removeEventListener('resize', this._onResize);
        }

        this.disposeObject3D(this.scene);
        if (this.renderer) {
            this.renderer.dispose();
        }
        if (this.controls) {
            this.controls.dispose();
        }

        // 从实例映射中删除
        Object.keys(Heatmap3DRenderer.instances).forEach(id=>{
            if (Heatmap3DRenderer.instances[id] === this) delete Heatmap3DRenderer.instances[id];
        });
    }

    static SIGNAL_BW_MHZ = {
        'FM Radio': 0.2,
        'ADS-B 1090': 1,
        'Airband (AM)': 0.025,
        'AIS / Marine': 0.025,
        'GSM 900 DL': 0.2,
        'GSM 900 UL': 0.2,
        'ISM 868': 0.125,
        'LoRa / ISM 433': 0.125,
        'LTE 1800': 20,
        'RC Aircraft': 0.05,
        'RC Ground': 0.05,
        'RC Low Band': 0.05,
        'TETRA / Emergency': 0.025
    };

    setSignalType(type){
        this.currentSignalType = type;
    }

    computeSpeedMbps(rssi){
        const bwMHz = Heatmap3DRenderer.SIGNAL_BW_MHZ[this.currentSignalType] || 1;
        const bandwidth = bwMHz * 1e6; // Hz
        const noiseFloor = -100; // dBm
        const snrDb = rssi - noiseFloor;
        const snrLin = Math.pow(10, snrDb/10);
        const capacity = bandwidth * Math.log2(1+snrLin); // bits/s
        const mbps = capacity / 1e6;
        return mbps.toFixed(2) + ' Mbps';
    }

    // 新增：将相机切换到俯视视角
    setTopView(distance = 10){
        // 将相机移至场景正上方，并稍微偏移 Z 轴避免 gimbal 锁
        this.camera.position.set(0, distance, 0.001);
        this.controls.target.set(0, 0, 0);
        this.controls.update();
    }
} 