import React, { useEffect, useState } from 'react';
import { wsService } from '../services/websocket';
import { useRosIntrospection } from '../services/store';
import { Activity, Network, Package, ServerCog, RefreshCw, Terminal } from 'lucide-react';

export default function NodeManager() {
  const [activeTab, setActiveTab] = useState('nodes');
  const [nodes, setNodes] = useState([]);
  const [topics, setTopics] = useState([]);
  const [packages, setPackages] = useState([]);
  const [loading, setLoading] = useState(false);
  const [selectedPkg, setSelectedPkg] = useState(null);
  const [executables, setExecutables] = useState([]);
  const [selectedNode, setSelectedNode] = useState(null);
  const [nodeParams, setNodeParams] = useState(null);
  const [paramEdit, setParamEdit] = useState({});
  
  const rosIntrospection = useRosIntrospection() || {};

  useEffect(() => {
    // Listen for custom introspection replies
    const unsubNodes = wsService.on('ros_list_nodes', data => {
      setNodes(data.nodes || []);
      setLoading(false);
    });
    const unsubTopics = wsService.on('ros_list_topics', data => {
      setTopics(data.topics || []);
      setLoading(false);
    });
    const unsubPkgs = wsService.on('ros_list_packages', data => {
      setPackages(data.packages || []);
      setLoading(false);
    });
    const unsubExecs = wsService.on('ros_list_executables', data => {
      if (data.package === selectedPkg) {
        setExecutables(data.executables || []);
      }
      setLoading(false);
    });
    const unsubParams = wsService.on('ros_node_params', data => {
      if (data.node_name === selectedNode) {
        setNodeParams(data.params || {});
      }
      setLoading(false);
    });

    fetchData();

    return () => {
      unsubNodes();
      unsubTopics();
      unsubPkgs();
      unsubExecs();
      unsubParams();
    };
  }, [selectedPkg, selectedNode]);

  const fetchData = () => {
    setLoading(true);
    if (activeTab === 'nodes') {
      wsService.send('ros_list_nodes', {});
      if (selectedNode) wsService.send('ros_node_params', { node_name: selectedNode });
    }
    else if (activeTab === 'topics') wsService.send('ros_list_topics', {});
    else if (activeTab === 'packages') {
      wsService.send('ros_list_packages', {});
      if (selectedPkg) wsService.send('ros_list_executables', { package: selectedPkg });
    }
  };

  useEffect(() => {
    fetchData();
  }, [activeTab]);

  return (
    <div className="page-container max-w-6xl mx-auto flex flex-col h-full">
      <div className="flex justify-between items-end mb-6 shrink-0">
        <div>
          <h1 className="page-title">ROS Introspection</h1>
          <p className="page-subtitle mb-0">Debug nodes, topics, and system packages.</p>
        </div>
        <button onClick={fetchData} disabled={loading} className="btn btn-secondary flex items-center gap-2">
          <RefreshCw size={16} className={loading ? 'animate-spin' : ''} /> REFRESH
        </button>
      </div>

      <div className="flex gap-4 mb-6 shrink-0">
        <button 
          onClick={() => setActiveTab('nodes')}
          className={`btn flex items-center gap-2 ${activeTab === 'nodes' ? 'btn-primary' : 'bg-[rgba(255,255,255,0.05)] border-transparent'}`}
        >
          <Activity size={18} /> Active Nodes
        </button>
        <button 
          onClick={() => setActiveTab('topics')}
          className={`btn flex items-center gap-2 ${activeTab === 'topics' ? 'btn-primary' : 'bg-[rgba(255,255,255,0.05)] border-transparent'}`}
        >
          <Network size={18} /> Topics
        </button>
        <button 
          onClick={() => setActiveTab('packages')}
          className={`btn flex items-center gap-2 ${activeTab === 'packages' ? 'btn-primary' : 'bg-[rgba(255,255,255,0.05)] border-transparent'}`}
        >
          <Package size={18} /> Packages & Executables
        </button>
      </div>

      <div className="card flex-1 flex flex-col min-h-0 overflow-hidden">
        <div className="flex-1 overflow-y-auto pr-2 custom-scrollbar">
          
          {activeTab === 'nodes' && (
            <div className="flex gap-4 h-full">
              <div className="flex-1 overflow-y-auto pr-2 custom-scrollbar grid grid-cols-1 md:grid-cols-2 gap-4 pb-4 h-max">
                {nodes.map((node, idx) => (
                  <div key={idx} 
                       className={`bg-[rgba(255,255,255,0.03)] border rounded p-4 cursor-pointer transition-colors ${selectedNode === node.name ? 'border-accent' : 'border-[rgba(255,255,255,0.05)] hover:border-[rgba(255,255,255,0.2)]'}`}
                       onClick={() => { setSelectedNode(node.name); setNodeParams(null); wsService.send('ros_node_params', { node_name: node.name }); }}>
                    <div className="flex items-center gap-3 mb-2">
                      <ServerCog size={20} className="text-accent" />
                      <h3 className="font-bold text-sm truncate" title={node.name}>{node.name}</h3>
                    </div>
                    <div className="text-xs text-dim font-mono truncate" title={node.namespace}>NS: {node.namespace}</div>
                  </div>
                ))}
                {nodes.length === 0 && !loading && <div className="text-muted p-4 col-span-full text-center">No nodes found. Is ROS running?</div>}
              </div>
              
              {/* Parameter Editor Panel */}
              {selectedNode && (
                <div className="w-1/3 bg-[rgba(0,0,0,0.2)] border-l border-[rgba(255,255,255,0.05)] p-4 overflow-y-auto flex flex-col custom-scrollbar">
                  <h3 className="font-bold mb-4 flex items-center gap-2 text-sm">
                    <Terminal size={16} /> Parameters: {selectedNode.split('/').pop()}
                  </h3>
                  {!nodeParams && <div className="text-muted text-sm animate-pulse">Loading parameters...</div>}
                  {nodeParams && Object.keys(nodeParams).length === 0 && <div className="text-muted text-sm">No parameters found.</div>}
                  {nodeParams && (
                    <div className="flex flex-col gap-3">
                      {Object.entries(nodeParams).map(([k, v]) => (
                        <div key={k} className="flex flex-col gap-1">
                          <label className="text-xs text-dim font-mono break-words">{k}</label>
                          <div className="flex gap-2">
                            <input 
                              type="text" 
                              className="input flex-1 font-mono text-sm"
                              value={paramEdit[k] !== undefined ? paramEdit[k] : String(v)}
                              onChange={e => setParamEdit({...paramEdit, [k]: e.target.value})}
                            />
                            {paramEdit[k] !== undefined && paramEdit[k] !== String(v) && (
                              <button 
                                className="btn btn-primary btn-sm px-2"
                                onClick={() => {
                                  if (confirm(`Set parameter ${k} to ${paramEdit[k]} for ${selectedNode}?`)) {
                                    wsService.send('set_parameters', { node: selectedNode, params: { [k]: paramEdit[k] } });
                                  }
                                }}
                              >
                                SET
                              </button>
                            )}
                          </div>
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              )}
            </div>
          )}

          {activeTab === 'topics' && (
            <div className="flex flex-col gap-2 pb-4">
              {topics.map((topic, idx) => (
                <div key={idx} className="bg-[rgba(255,255,255,0.03)] border border-[rgba(255,255,255,0.05)] rounded px-4 py-3 flex flex-col sm:flex-row sm:items-center justify-between hover:bg-[rgba(255,255,255,0.05)] transition-colors">
                  <div className="font-mono text-sm text-text font-bold mb-1 sm:mb-0">{topic.name}</div>
                  <div className="flex gap-2 flex-wrap">
                    {topic.types.map((t, i) => (
                      <span key={i} className="badge badge-info text-xs">{t}</span>
                    ))}
                  </div>
                </div>
              ))}
              {topics.length === 0 && !loading && <div className="text-muted p-4 text-center">No topics found.</div>}
            </div>
          )}

          {activeTab === 'packages' && (
            <div className="flex gap-4 h-full">
              <div className="flex-1 overflow-y-auto pr-2 custom-scrollbar grid grid-cols-1 md:grid-cols-2 gap-4 pb-4 h-max">
                {packages.map((pkg, idx) => (
                  <div key={idx} 
                       className={`bg-[rgba(255,255,255,0.03)] border rounded p-4 flex flex-col gap-1 cursor-pointer transition-colors ${selectedPkg === pkg.name ? 'border-accent' : 'border-[rgba(255,255,255,0.05)] hover:border-[rgba(255,255,255,0.2)]'}`}
                       onClick={() => { setSelectedPkg(pkg.name); setExecutables([]); wsService.send('ros_list_executables', { package: pkg.name }); }}>
                    <h3 className="font-bold text-sm text-accent">{pkg.name}</h3>
                    <div className="text-xs text-dim font-mono truncate" title={pkg.path}>{pkg.path}</div>
                  </div>
                ))}
                {packages.length === 0 && !loading && <div className="text-muted p-4 col-span-full text-center">No packages found.</div>}
              </div>
              
              {/* Executables Panel */}
              {selectedPkg && (
                <div className="w-1/3 bg-[rgba(0,0,0,0.2)] border-l border-[rgba(255,255,255,0.05)] p-4 overflow-y-auto flex flex-col custom-scrollbar">
                  <h3 className="font-bold mb-4 flex items-center gap-2 text-sm">
                    <Terminal size={16} /> Executables: {selectedPkg}
                  </h3>
                  {executables.length === 0 && !loading && <div className="text-muted text-sm">No executables found.</div>}
                  {executables.length === 0 && loading && <div className="text-muted text-sm animate-pulse">Loading...</div>}
                  <div className="flex flex-col gap-2">
                    {executables.map((exe, i) => (
                      <div key={i} className="bg-[rgba(255,255,255,0.02)] p-2 rounded text-sm font-mono text-text flex items-center justify-between border border-[rgba(255,255,255,0.05)]">
                        <span className="truncate" title={exe}>{exe}</span>
                        <button 
                          className="btn btn-secondary btn-sm"
                          onClick={() => alert(`To run ${exe}, please configure it in config/processes.yaml and use the Controllers page for safety.`)}
                        >
                          RUN
                        </button>
                      </div>
                    ))}
                  </div>
                </div>
              )}
            </div>
          )}

        </div>
      </div>
    </div>
  );
}
